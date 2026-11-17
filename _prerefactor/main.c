/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : LiveArrow - smart arrow firmware.
 *                      CH32V005D6U6 + ISM330DHCX, LED on PD0.
 *
 *                      Launch trigger: the sensor's wake-up engine watches for
 *                      a high-g event and asserts INT1; the MCU sleeps until
 *                      then, and confirms the arrow is actually in flight
 *                      before lighting the LED.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 * Why two stages.
 *
 *   A compound bow accelerates the arrow to ~90 m/s over a ~0.6 m power
 *   stroke: a = v^2/2d ~= 690 g, lasting ~13 ms. The accelerometer's full
 *   scale is 16 g, so the launch does not get measured - it pegs the ADC.
 *   That makes a high-g spike a poor discriminator on its own: dropping the
 *   arrow on concrete also pegs it.
 *
 *   What is specific to flight is what happens AFTER the spike. Once the
 *   arrow leaves the string nothing opposes gravity except drag, so the
 *   accelerometer - which measures specific force, not acceleration -
 *   collapses from a rock-steady 1.000 g to roughly 0.6 g and falling.
 *   Fletching also spins the shaft.
 *
 *   So: hardware wake on >12 g (cheap, lets the MCU sleep), then confirm in
 *   software that the low-g state is SUSTAINED. A knock spikes and returns
 *   to 1 g immediately; a drop is short; a shot lasts 1-3 seconds.
 */

#include "debug.h"
#include "LED.h"
#include "IMU.h"

/* ===================== tuning =========================================== */

/* Scale factors for +/-16 g and +/-4000 dps. */
#define G_COUNTS                2049        /* 1 g   (0.488 mg/LSB)        */
#define DPS_COUNTS              7           /* 1 dps (140 mdps/LSB), ~7.14 */

/* Which sensor axis runs along the arrow shaft. Confirm with the tilt test:
 * stand the arrow on its nock and see which accel axis reads ~+/-2049.
 * Used for the roll-rate check. */
#define SHAFT_AXIS              0           /* 0=X, 1=Y, 2=Z               */

/* In flight the accelerometer sees drag only: ~0.6 g at launch speed,
 * falling as v^2. At rest it reads 1.000 g to within a few mg. */
#define BALLISTIC_COUNTS        ((7 * G_COUNTS) / 10)       /* 1434 = 0.7 g */

/* ---------------------- rotational (roll) criterion ---------------------
 *
 * Helical fletching acts as a screw thread in air: the revolutions per metre
 * travelled are fixed by the fletching geometry, so the rotation rate is
 * proportional to airspeed. That proportionality is what separates events:
 *
 *     shot from a bow     ~90 m/s   10-20 rev/s   3600-7200 dps
 *     hard throw by hand  ~25 m/s    3-6  rev/s   1000-2000 dps
 *     manual handling       -                      300-500  dps
 *     stationary            -                      under 1  dps  (measured)
 *
 * Two independent tests are applied, and BOTH must pass for the roll
 * criterion to be considered met:
 *
 *   rate    the rate must stay above ROLL_RATE_DPS for ROLL_RATE_SAMPLES
 *           consecutive samples. Sustained, not peak - a single noisy
 *           sample must not qualify.
 *
 *   angle   the rotation accumulated across the whole window must reach
 *           ROLL_ANGLE_DEG. This is the stronger of the two: it integrates
 *           evidence over time and is immune to brief spikes. A shot at
 *           3600 dps accumulates 2160 degrees in 600 ms; a throw at
 *           1500 dps accumulates 900; hand motion accumulates under 300.
 */
#define ROLL_RATE_DPS           2000u   /* below the 3600 dps shot floor,   */
                                        /* above the ~1500 dps of a throw   */
#define ROLL_RATE_COUNTS        (ROLL_RATE_DPS * DPS_COUNTS)
#define ROLL_RATE_SAMPLES       10u     /* 10 x 5 ms = 50 ms sustained      */
#define ROLL_ANGLE_DEG          1080u   /* three full revolutions           */

/* Angle is accumulated as a running sum of |rate| in raw counts, and the
 * threshold is converted into that same unit, so no division or scaling is
 * needed on the sampling path - this part has no hardware divider.
 *
 *   degrees = counts x 0.140 dps x (CONFIRM_PERIOD_MS / 1000) s
 * so one degree corresponds to this many accumulated counts: */
#define ROLL_SUM_PER_DEG        (1000000u / (140u * CONFIRM_PERIOD_MS))
#define ROLL_ANGLE_SUM          (ROLL_ANGLE_DEG * ROLL_SUM_PER_DEG)

/* How the roll criterion participates in the decision.
 *
 *   REPORT   measured and printed, but decides nothing
 *   REQUIRE  ballistic AND roll - strictest; rejects a hard throw, but a
 *            shot from straight-fletched arrows would also be rejected
 *   EITHER   ballistic OR roll - roll can only ADD detections, never block
 *            one. Protects against the ballistic threshold being wrong,
 *            since that figure is still an estimate
 *   ONLY     roll alone decides; the ballistic test is ignored
 *
 * EITHER is the default because of the cost asymmetry: a missed shot loses
 * the arrow, a spurious activation only wastes battery. Do not move to
 * REQUIRE until the spin your fletching actually produces is measured. */
#define ROLL_MODE_REPORT        0
#define ROLL_MODE_REQUIRE       1
#define ROLL_MODE_EITHER        2
#define ROLL_MODE_ONLY          3

#define ROLL_MODE               ROLL_MODE_EITHER

/* Print per-axis roll evidence after every candidate. Use this to identify
 * which axis actually lies along the shaft before trusting SHAFT_AXIS. */
#define ROLL_DIAG               1

/* Confirmation window. One sample per CONFIRM_PERIOD_MS; the sensor ODR is
 * 208 Hz (4.8 ms) so 5 ms is about one fresh sample per read.
 *
 * BALLISTIC_SAMPLES = 0 means LAUNCH DETECTION ONLY: the >12 g wake fires
 * the LED straight away, with no check that the arrow is actually flying.
 * Fastest possible response, but the wake threshold becomes the only gate -
 * anything that hits it (notably dropping the arrow on a hard floor, where
 * the IMPACT trips the wake) will light the LED.
 *
 *   0   trigger on the launch spike alone            (~5 ms latency)
 *   10  50 ms of free-fall - rejects a static knock  (~55 ms)
 *   60  300 ms - full flight confirmation            (~315 ms)
 */
#define CONFIRM_PERIOD_MS       5
#define BALLISTIC_SAMPLES       10          /* 50 ms of free-fall           */
#define CONFIRM_TIMEOUT         120         /* give up after 600 ms         */

/* The confirmation window runs whenever any criterion needs it. With no
 * ballistic requirement and roll reporting only, there is nothing to
 * evaluate and the wake fires the indicator directly, as before. */
#define NEEDS_CONFIRM   ((BALLISTIC_SAMPLES > 0) || (ROLL_MODE != ROLL_MODE_REPORT))

#if (ROLL_MODE == ROLL_MODE_REQUIRE) && (BALLISTIC_SAMPLES == 0)
#error "ROLL_MODE_REQUIRE needs a ballistic requirement; set BALLISTIC_SAMPLES > 0"
#endif
#if (ROLL_RATE_SAMPLES > CONFIRM_TIMEOUT)
#error "ROLL_RATE_SAMPLES cannot exceed the confirmation window"
#endif

/* What the indicator does once triggered.
 *
 * The sequence itself - a continuous phase, then a repeating flash, both
 * duty-limited to reduce average current - is configured in LED.c and is
 * produced by the TIM2 interrupt. This file only starts it.
 *
 * LED_LATCH 1 halts the firmware after triggering, so the unit indicates
 * once and never re-arms. */
#define LED_LATCH               0

/* Having started the indication, wait this long before re-arming, so the
 * landing impact cannot immediately produce a second trigger. Flight lasts
 * roughly 1-3 s, so the impact falls inside this window. The indication
 * continues in the background throughout. */
#define REARM_LOCKOUT_MS        3000

/* ===================== power ============================================
 *
 * USE_SLEEP    0 - poll g_imu_int_flag in a loop (highest current, easiest
 *                  to debug, and the only mode where ARMED_HEARTBEAT works)
 *              1 - __WFI(): the core halts until INT1 fires and then resumes
 *                  at the next instruction. Peripherals and the PLL keep
 *                  running. State is preserved. Debugger stays connected.
 *
 * USE_STANDBY  1 - deep sleep instead: PDDS + SLEEPDEEP, microamps rather
 *                  than milliamps. Only meaningful when USE_SLEEP is 1.
 *
 * Standby notes, all of which bite if ignored:
 *   - RCC_PB1Periph_PWR must be clocked before any PWR register is touched.
 *   - The "Disable Standby-Mode RST" option byte must be set in
 *     WCH-LinkUtility, otherwise entering standby resets the chip.
 *   - SWIO goes away in standby, so the debugger disconnects and SDI printf
 *     will misbehave. Test standby with the LED, or over the PA2 UART.
 *   - EXTI needs no extra setup: WCH's __WFE (which __WFI calls) copies
 *     EXTI->INTENR into EXTI->EVENR whenever SLEEPDEEP is set, because the
 *     core wakes on events rather than interrupts in deep sleep. It restores
 *     them and re-enables IRQs on the way out, so the ISR still runs and
 *     PWR_EnterSTANDBYMode() returns to its caller.
 */
#define USE_SLEEP               1
#define USE_STANDBY             0

/* Short flash every few seconds while armed, so you can see it is alive.
 * Incompatible with sleeping - the loop has to keep running to time it. */
#define ARMED_HEARTBEAT         0
#define HEARTBEAT_PERIOD_MS     3000

#if USE_SLEEP && ARMED_HEARTBEAT
#error "ARMED_HEARTBEAT needs a running loop; set one of them to 0"
#endif

/* ======================================================================== */

typedef enum
{
    STATE_ARMED = 0,
    STATE_CANDIDATE,
    STATE_FLYING
} arrow_state_t;

/* Largest absolute component. Used instead of a true magnitude because the
 * launch saturates at 32767 counts and 3 x 32767^2 overflows int32_t. */
static uint16_t max_abs3(const int16_t v[3])
{
    uint16_t m = 0;
    uint8_t  i;
    uint32_t a;

    for (i = 0; i < 3u; i++)
    {
        a = (v[i] < 0) ? (uint32_t)(-(int32_t)v[i]) : (uint32_t)v[i];
        if (a > m) { m = (uint16_t)a; }
    }
    return m;
}

/* True when the total specific force is below BALLISTIC_COUNTS.
 * The max_abs guard keeps the sum-of-squares safely inside int32_t: any
 * single component >= the threshold already means the magnitude is too big,
 * so we never square a large number. No sqrt needed - compare squares. */
static uint8_t is_ballistic(const int16_t a[3])
{
    int32_t mag2;

    if (max_abs3(a) >= (uint16_t)BALLISTIC_COUNTS)
    {
        return 0;
    }

    mag2 = (int32_t)a[0] * a[0]
         + (int32_t)a[1] * a[1]
         + (int32_t)a[2] * a[2];

    return (mag2 < ((int32_t)BALLISTIC_COUNTS * BALLISTIC_COUNTS)) ? 1u : 0u;
}

static uint16_t abs_rate(int16_t w)
{
    int32_t r = w;
    if (r < 0) { r = -r; }
    return (uint16_t)r;
}

/* Rotation evidence gathered across one candidate window. */
typedef struct
{
    uint16_t peak[3];       /* peak |rate| per axis, raw counts            */
    uint32_t sum[3];        /* accumulated |rate| per axis - proportional  */
                            /* to total rotation angle about that axis     */
    uint16_t run;           /* consecutive samples above the rate threshold */
    uint16_t best_run;      /* longest such run this window                 */
} roll_t;

static void roll_reset(roll_t *r)
{
    uint8_t i;
    for (i = 0; i < 3u; i++) { r->peak[i] = 0; r->sum[i] = 0; }
    r->run = 0;
    r->best_run = 0;
}

static void roll_update(roll_t *r, const int16_t gyro[3])
{
    uint16_t w;
    uint8_t  i;

    for (i = 0; i < 3u; i++)
    {
        w = abs_rate(gyro[i]);
        if (w > r->peak[i]) { r->peak[i] = w; }
        r->sum[i] += w;
    }

    /* Only the shaft axis decides. Requiring rotation specifically about
     * the shaft is what separates spin-stabilised flight from an object
     * that is simply tumbling. */
    if (abs_rate(gyro[SHAFT_AXIS]) >= (uint16_t)ROLL_RATE_COUNTS)
    {
        r->run++;
        if (r->run > r->best_run) { r->best_run = r->run; }
    }
    else
    {
        r->run = 0;         /* sustained, not cumulative */
    }
}

/* Both tests must pass: a sustained rate AND enough accumulated rotation. */
static uint8_t roll_met(const roll_t *r)
{
    return ((r->best_run >= (uint16_t)ROLL_RATE_SAMPLES) &&
            (r->sum[SHAFT_AXIS] >= (uint32_t)ROLL_ANGLE_SUM)) ? 1u : 0u;
}

/* Accumulated rotation about one axis, in whole degrees. Division only -
 * this is the reporting path, never the sampling path. */
static uint32_t roll_degrees(const roll_t *r, uint8_t axis)
{
    return r->sum[axis] / (uint32_t)ROLL_SUM_PER_DEG;
}

/* Halt the core until an interrupt arrives. There is no lost-wakeup race:
 * on RISC-V, wfi returns immediately when an interrupt is already pending,
 * so an INT1 landing between the flag test and this call is not missed. */
static void mcu_sleep(void)
{
#if USE_SLEEP
#if USE_STANDBY
    PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFI);
#else
    __WFI();
#endif
#else
    Delay_Ms(20);
#endif
}

int main(void)
{
    uint8_t       whoami;
    uint8_t       src;
    int16_t       gyro[3];
    int16_t       accel[3];
    arrow_state_t state = STATE_ARMED;

    uint16_t      run       = 0;    /* consecutive ballistic samples        */
    uint16_t      best_run  = 0;    /* longest run seen this candidate      */
    uint16_t      ticks     = 0;    /* samples elapsed in the window        */
    uint32_t      shots     = 0;
    roll_t        roll;             /* rotation evidence this candidate     */
    uint8_t       ballistic_ok = 0;
    uint8_t       roll_ok      = 0;
#if ARMED_HEARTBEAT
    uint32_t      hb        = 0;    /* heartbeat timer, ms                  */
#endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("\r\n=== LiveArrow ===\r\n");
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

#if USE_SLEEP && USE_STANDBY
    /* Must be clocked before PWR_EnterSTANDBYMode touches PWR->CTLR. */
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_PWR, ENABLE);
#endif

    init_LED();
    init_IMU();

    whoami = imu_who_am_i();
    printf("WHO_AM_I: 0x%02x (expect 0x6B)\r\n", whoami);

    if (whoami != 0x6B)
    {
        printf("Bus not working - see IMU.c notes.\r\n");
        imu_find_remap();
        while (1) { LED_Toggle(); Delay_Ms(100); }
    }

    imu_config();

    printf("\r\nTrigger armed.\r\n");
    printf("  wake      > 15 g          (hardware, WAKE_UP_THS)\r\n");
#if !NEEDS_CONFIRM
    printf("  confirm     none - launch spike fires the LED directly\r\n");
#else
#if (BALLISTIC_SAMPLES > 0)
    printf("  ballistic < %d counts     (%d.%d g), %d ms sustained\r\n",
           BALLISTIC_COUNTS,
           BALLISTIC_COUNTS / G_COUNTS,
           ((BALLISTIC_COUNTS * 10) / G_COUNTS) % 10,
           BALLISTIC_SAMPLES * CONFIRM_PERIOD_MS);
#endif
    printf("  roll rate > %u dps        sustained %u ms\r\n",
           (unsigned int)ROLL_RATE_DPS,
           (unsigned int)(ROLL_RATE_SAMPLES * CONFIRM_PERIOD_MS));
    printf("  roll angle> %u deg        accumulated on axis %d\r\n",
           (unsigned int)ROLL_ANGLE_DEG, SHAFT_AXIS);
    printf("  roll mode   %s\r\n",
           (ROLL_MODE == ROLL_MODE_REPORT)  ? "report only" :
           (ROLL_MODE == ROLL_MODE_REQUIRE) ? "ballistic AND roll" :
           (ROLL_MODE == ROLL_MODE_EITHER)  ? "ballistic OR roll" :
                                              "roll only");
    printf("  timeout     %d ms\r\n", CONFIRM_TIMEOUT * CONFIRM_PERIOD_MS);
#endif
    printf("  indicator   30 s continuous, then 3 s in every 30 s\r\n");
    printf("                duty-limited, see LED.c\r\n");
#if USE_SLEEP
#if USE_STANDBY
    printf("  idle        STANDBY (deep sleep, debugger will drop)\r\n\r\n");
#else
    printf("  idle        __WFI sleep, woken by INT1\r\n\r\n");
#endif
#else
    printf("  idle        polling (no sleep)\r\n\r\n");
#endif

    LED_Off();

    while (1)
    {
        switch (state)
        {
        /* ---------------------------------------------------------------
         * ARMED - idle. The sensor watches for >12 g on its own, so there
         * is nothing to poll: halt the core and let INT1 restart it.
         * --------------------------------------------------------------- */
        case STATE_ARMED:
            if (g_imu_int_flag == 0)
            {
                mcu_sleep();            /* core stops here until INT1 */
            }

            if (g_imu_int_flag != 0)
            {
                g_imu_int_flag = 0;

                /* Reading WAKE_UP_SRC releases the latched INT1. Without
                 * this the pin stays high and never fires again. */
                src = imu_clear_int();

                run      = 0;
                best_run = 0;
                ticks    = 0;
                roll_reset(&roll);

#if !NEEDS_CONFIRM
                /* Nothing to evaluate - the wake event IS the trigger. */
                state = STATE_FLYING;
                printf("wake  src=0x%02x -> launch\r\n", src);
#else
                state = STATE_CANDIDATE;
                printf("wake  src=0x%02x -> confirming...\r\n", src);
#endif
            }
#if ARMED_HEARTBEAT
            else
            {
                hb += 20;
                if (hb >= HEARTBEAT_PERIOD_MS)
                {
                    hb = 0;
                    LED_On();  Delay_Ms(20);  LED_Off();
                }
            }
#endif
            break;

        /* ---------------------------------------------------------------
         * CANDIDATE - something hit hard. Was it a shot, or a knock?
         * Look for a SUSTAINED low-g run: a knock returns to 1 g within a
         * sample or two, a shot stays ballistic for the whole flight.
         * --------------------------------------------------------------- */
        case STATE_CANDIDATE:
#if !NEEDS_CONFIRM
            state = STATE_FLYING;           /* unreachable, kept for safety */
#else
            imu_read_raw(gyro, accel);
            ticks++;

            roll_update(&roll, gyro);

            if (is_ballistic(accel) != 0)
            {
                run++;
                if (run > best_run) { best_run = run; }
            }
            else
            {
                run = 0;            /* must be consecutive */
            }

            /* Evaluate both criteria every sample, so whichever is satisfied
             * first ends the window. The ballistic test usually wins on a
             * normal shot; the rotational test is what still catches one if
             * the drag estimate behind the ballistic threshold is wrong. */
#if (BALLISTIC_SAMPLES > 0)
            ballistic_ok = (run >= (uint16_t)BALLISTIC_SAMPLES) ? 1u : 0u;
#else
            ballistic_ok = 0u;
#endif
            roll_ok = roll_met(&roll);

#if   (ROLL_MODE == ROLL_MODE_REPORT)
            if (ballistic_ok)                      { state = STATE_FLYING; }
#elif (ROLL_MODE == ROLL_MODE_REQUIRE)
            if (ballistic_ok && roll_ok)           { state = STATE_FLYING; }
#elif (ROLL_MODE == ROLL_MODE_EITHER)
            if (ballistic_ok || roll_ok)           { state = STATE_FLYING; }
#else /* ROLL_MODE_ONLY */
            if (roll_ok)                           { state = STATE_FLYING; }
#endif

            if ((state == STATE_CANDIDATE) && (ticks >= CONFIRM_TIMEOUT))
            {
                printf("  rejected: ballistic run %u/%u, roll rate run %u/%u"
                       " (peak %u dps), roll angle %u/%u deg\r\n",
                       (unsigned int)best_run, (unsigned int)BALLISTIC_SAMPLES,
                       (unsigned int)roll.best_run, (unsigned int)ROLL_RATE_SAMPLES,
                       (unsigned int)(roll.peak[SHAFT_AXIS] / DPS_COUNTS),
                       (unsigned int)roll_degrees(&roll, SHAFT_AXIS),
                       (unsigned int)ROLL_ANGLE_DEG);
#if ROLL_DIAG
                printf("            per-axis deg  X=%u  Y=%u  Z=%u"
                       "   peak dps  X=%u  Y=%u  Z=%u\r\n",
                       (unsigned int)roll_degrees(&roll, 0),
                       (unsigned int)roll_degrees(&roll, 1),
                       (unsigned int)roll_degrees(&roll, 2),
                       (unsigned int)(roll.peak[0] / DPS_COUNTS),
                       (unsigned int)(roll.peak[1] / DPS_COUNTS),
                       (unsigned int)(roll.peak[2] / DPS_COUNTS));
#endif
                g_imu_int_flag = 0;         /* drop anything queued */
                (void)imu_clear_int();
                state = STATE_ARMED;
            }

            if (state == STATE_CANDIDATE)
            {
                Delay_Ms(CONFIRM_PERIOD_MS);
            }
#endif  /* NEEDS_CONFIRM */
            break;

        /* ---------------------------------------------------------------
         * FLYING - confirmed. Light it.
         * --------------------------------------------------------------- */
        case STATE_FLYING:
            shots++;
            printf("*** SHOT %u ***  accepted by %s\r\n",
                   (unsigned int)shots,
                   (ballistic_ok && roll_ok) ? "ballistic + roll" :
                   ballistic_ok              ? "ballistic" :
                   roll_ok                   ? "roll" : "wake");
            printf("              ballistic run %u/%u   roll rate run %u/%u"
                   " (peak %u dps)   roll angle %u/%u deg\r\n",
                   (unsigned int)best_run, (unsigned int)BALLISTIC_SAMPLES,
                   (unsigned int)roll.best_run, (unsigned int)ROLL_RATE_SAMPLES,
                   (unsigned int)(roll.peak[SHAFT_AXIS] / DPS_COUNTS),
                   (unsigned int)roll_degrees(&roll, SHAFT_AXIS),
                   (unsigned int)ROLL_ANGLE_DEG);
#if ROLL_DIAG
            printf("              per-axis deg  X=%u  Y=%u  Z=%u\r\n",
                   (unsigned int)roll_degrees(&roll, 0),
                   (unsigned int)roll_degrees(&roll, 1),
                   (unsigned int)roll_degrees(&roll, 2));
#endif

            /* Returns immediately. The continuous phase and the repeating
             * flash that follows are produced by the TIM2 interrupt, so the
             * state machine is free to re-arm while the indicator is still
             * running - no 60-second blocking delay in the main loop. */
            LED_Trigger();

#if LED_LATCH
            while (1) { }                   /* indicate once, never re-arm */
#else
            Delay_Ms(REARM_LOCKOUT_MS);     /* ride out the landing impact */
            g_imu_int_flag = 0;
            (void)imu_clear_int();
            state = STATE_ARMED;
            printf("re-armed (indication still running)\r\n\r\n");
#endif
            break;

        default:
            state = STATE_ARMED;
            break;
        }
    }
}
