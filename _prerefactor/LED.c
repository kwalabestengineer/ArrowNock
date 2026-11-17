/*
 * LED.c - indicator: brightness limiting and post-launch flash pattern.
 *
 * Hardware:
 *   PD0  indicator drive.  D1/D2 anodes sit on +3V3 through 330R series
 *        resistors, so the pin SINKS current: driving it LOW lights the
 *        indicator. That inversion is confined to this file.
 *
 * Why the duty limiting is produced in software rather than by a timer
 * output:
 *   PD0 is TIM1 channel 1 complementary output, so hardware PWM would in
 *   principle be possible. It would require a TIM1 pin mapping that places
 *   that output on PD0, and the equivalent assumption for the serial
 *   interface proved false on this board - no available mapping produced a
 *   working bus, which is why the sensor transport is also implemented in
 *   software. Rather than repeat that assumption, TIM2 is used purely as a
 *   time base and the pin is driven directly. The cost is two interrupts
 *   per period, approximately 0.2 % of the processor at the default
 *   settings. Hardware PWM remains available as a later optimisation if a
 *   suitable mapping is confirmed from the reference manual.
 */
#include "LED.h"
#include <ch32v00X.h>

/* ------------------------------------------------------------------------
 * Pin - private to this module.
 * ------------------------------------------------------------------------ */
#define LED_PORT            GPIOD
#define LED_PIN             GPIO_Pin_0
#define LED_RCC             RCC_PB2Periph_GPIOD

#define LED_DRIVE_ON()      GPIO_ResetBits(LED_PORT, LED_PIN)   /* active low */
#define LED_DRIVE_OFF()     GPIO_SetBits(LED_PORT, LED_PIN)

/* ========================================================================
 * Tuning
 * ======================================================================== */

/* Duty-limiting frequency.
 *
 * 1000 Hz is the default rather than a lower value because this indicator
 * is attached to an arrow in flight. At 90 m/s the arrow covers 0.9 m in
 * one period at 100 Hz, so a 70 % duty would paint the flight path as a
 * dashed line of roughly 0.63 m marks separated by 0.27 m gaps - clearly
 * visible, and the opposite of the intent. At 1000 Hz the same gaps are
 * about 2.7 cm and the trace reads as continuous.
 *
 * Set to 100 for the originally specified value; the pattern timing below
 * is derived from this constant and stays correct either way. */
#define LED_PWM_FREQ_HZ     1000u

/* Fraction of each period the indicator is driven. Average current scales
 * with this directly; apparent brightness falls considerably less, because
 * the eye's response to luminance is compressed. */
#define LED_DUTY_PCT        70u

/* Post-launch pattern. */
#define LED_SOLID_S         30u     /* continuous immediately after launch */
#define LED_BLINK_ON_S      3u      /* lit for this long ...               */
#define LED_BLINK_PERIOD_S  30u     /* ... in every period, thereafter     */

/* ========================================================================
 * Derived timer constants
 * ======================================================================== */

/* TIM2 is prescaled to a 1 MHz tick so that the period and compare values
 * are readable directly in microseconds. The prescaler is computed from
 * SystemCoreClock, so the timing survives a change of system clock. */
#define LED_TIM_CLK_HZ      1000000u
#define LED_TIM_PERIOD      (LED_TIM_CLK_HZ / LED_PWM_FREQ_HZ)
#define LED_TIM_ARR         (LED_TIM_PERIOD - 1u)
#define LED_TIM_CCR         ((LED_TIM_PERIOD * LED_DUTY_PCT) / 100u)

/* One pattern tick per PWM period. */
#define LED_TICKS_PER_S     LED_PWM_FREQ_HZ
#define LED_SOLID_TICKS     (LED_SOLID_S * LED_TICKS_PER_S)
#define LED_BLINK_ON_TICKS  (LED_BLINK_ON_S * LED_TICKS_PER_S)
#define LED_BLINK_PER_TICKS (LED_BLINK_PERIOD_S * LED_TICKS_PER_S)

#if (LED_DUTY_PCT > 100u)
#error "LED_DUTY_PCT must be 0..100"
#endif
#if (LED_BLINK_ON_S > LED_BLINK_PERIOD_S)
#error "LED_BLINK_ON_S cannot exceed LED_BLINK_PERIOD_S"
#endif
#if (LED_TIM_ARR > 65535u)
#error "LED_PWM_FREQ_HZ too low: the 16-bit timer period would overflow"
#endif

/* ========================================================================
 * Module state
 * ======================================================================== */
typedef enum
{
    LED_MODE_DIRECT = 0,        /* caller owns the pin; timer stopped      */
    LED_MODE_PATTERN            /* the interrupt owns the pin              */
} led_mode_t;

static volatile led_mode_t led_mode  = LED_MODE_DIRECT;
static volatile uint8_t    led_lit   = 0;   /* is this period a lit one?   */
static volatile uint32_t   solid_left = 0;  /* ticks remaining in phase 1  */
static volatile uint32_t   phase_tick = 0;  /* position within phase 2     */

/* ========================================================================
 * Pattern
 * ======================================================================== */

/* Advance one PWM period and decide whether this period is lit.
 *
 * Deliberately written without division or modulo: this part has no
 * hardware divider, so a modulo here would call the software division
 * routine once per period. Two counters cost a few instructions instead. */
static void led_pattern_tick(void)
{
    if (solid_left != 0u)
    {
        solid_left--;                       /* phase 1: continuous */
        led_lit = 1u;
        if (solid_left == 0u)
        {
            phase_tick = 0u;                /* enter phase 2 cleanly */
        }
        return;
    }

    led_lit = (phase_tick < LED_BLINK_ON_TICKS) ? 1u : 0u;

    phase_tick++;
    if (phase_tick >= LED_BLINK_PER_TICKS)
    {
        phase_tick = 0u;
    }
}

/* ========================================================================
 * Bring-up
 * ======================================================================== */

static void led_timer_init(void)
{
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef       oc = {0};
    NVIC_InitTypeDef        nv = {0};

    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);

    tb.TIM_Prescaler         = (uint16_t)((SystemCoreClock / LED_TIM_CLK_HZ) - 1u);
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = (uint16_t)LED_TIM_ARR;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    /* Channel 1 is used only to generate a compare interrupt partway
     * through each period - the duty edge. No pin is driven by the timer,
     * hence Timing mode with the output stage disabled. */
    oc.TIM_OCMode      = TIM_OCMode_Timing;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_Pulse       = (uint16_t)LED_TIM_CCR;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);

    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, DISABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update | TIM_IT_CC1);

    nv.NVIC_IRQChannel                   = TIM2_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 2;
    nv.NVIC_IRQChannelSubPriority        = 2;
    nv.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nv);

    TIM_Cmd(TIM2, DISABLE);     /* started only while a pattern is running */
}

void init_LED(void)
{
    GPIO_InitTypeDef g = {0};

    RCC_PB2PeriphClockCmd(LED_RCC, ENABLE);

    /* Preload the output register high BEFORE the pin becomes an output.
     * It resets to zero, so configuring the pin first would drive it low
     * and flash the indicator at every reset. */
    GPIO_SetBits(LED_PORT, LED_PIN);

    g.GPIO_Pin   = LED_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(LED_PORT, &g);

    led_timer_init();

    led_mode = LED_MODE_DIRECT;
    LED_DRIVE_OFF();
}

/* ========================================================================
 * Public interface
 * ======================================================================== */

/* Stop the pattern and hand the pin back to direct control. Stopping the
 * timer matters: while the processor sleeps between launches, a running
 * TIM2 would wake it on every period and defeat the halt. */
static void led_stop_pattern(void)
{
    TIM_Cmd(TIM2, DISABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, DISABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update | TIM_IT_CC1);
    led_mode   = LED_MODE_DIRECT;
    led_lit    = 0u;
    solid_left = 0u;
    phase_tick = 0u;
}

void LED_On(void)
{
    led_stop_pattern();
    LED_DRIVE_ON();
}

void LED_Off(void)
{
    led_stop_pattern();
    LED_DRIVE_OFF();
}

void LED_Toggle(void)
{
    led_stop_pattern();
    if (GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) == Bit_RESET)
    {
        LED_DRIVE_OFF();        /* pin low  => lit  => extinguish */
    }
    else
    {
        LED_DRIVE_ON();         /* pin high => dark => light      */
    }
}

void LED_Trigger(void)
{
    __disable_irq();            /* the interrupt reads all of these */
    solid_left = LED_SOLID_TICKS;
    phase_tick = 0u;
    led_lit    = 1u;
    led_mode   = LED_MODE_PATTERN;
    __enable_irq();

    LED_DRIVE_ON();             /* light immediately, do not wait a period */

    TIM_SetCounter(TIM2, 0);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update | TIM_IT_CC1);
#if (LED_DUTY_PCT >= 100u)
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);          /* no duty edge */
#else
    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, ENABLE);
#endif
    TIM_Cmd(TIM2, ENABLE);
}

uint8_t LED_PatternActive(void)
{
    return (led_mode == LED_MODE_PATTERN) ? 1u : 0u;
}

/* ========================================================================
 * Interrupt
 *
 * Two events per period:
 *   update     start of the period - advance the pattern, light if lit
 *   compare    the duty edge        - extinguish for the remainder
 * ======================================================================== */
void LED_TimerISR(void)
{
    uint8_t is_update = 0u;
    uint8_t is_edge   = 0u;

    /* Flags are cleared unconditionally. Leaving one set because the mode
     * changed underneath would re-enter this handler indefinitely. */
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        is_update = 1u;
    }
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        is_edge = 1u;
    }

    if (led_mode != LED_MODE_PATTERN)
    {
        return;                 /* direct control owns the pin */
    }

    if (is_update != 0u)
    {
        led_pattern_tick();
        if (led_lit != 0u) { LED_DRIVE_ON(); } else { LED_DRIVE_OFF(); }
    }

    if ((is_edge != 0u) && (led_lit != 0u))
    {
        LED_DRIVE_OFF();
    }
}
