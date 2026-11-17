/*
 * main.c - LiveArrow baseline.
 *
 * What this program does, in order:
 *
 *   1. brings up the clock, the delay timer and the indicator
 *   2. arms SDI printf (which never blocks - see Debug/debug.c)
 *   3. brings up the IMU pins and INT1 -> EXTI line 0
 *   4. checks the sensor answers, and configures it
 *   5. loops: report any INT1 event, then read and print all six axes
 *
 * There is no launch detection here. This is the layer everything else
 * would be built on: working bus, working interrupt, working data.
 *
 * THE INDICATOR IS THE PRIMARY STATUS CHANNEL. printf only reaches a
 * debugger; on battery there is nobody listening. init_LED() therefore runs
 * before anything else, and each stage blinks a count so the board can be
 * diagnosed with no probe attached:
 *
 *   1 blink     reset done, clock and indicator up
 *   2 blinks    sensor answered and is configured - entering the main loop
 *   continuous  sensor did not answer; the bus is not working
 */
#include "debug.h"
#include "LED.h"
#include "IMU.h"

/* How often the main loop reads the sensor. The wake-up interrupt is
 * independent of this and is handled as soon as the loop comes round. */
#define SAMPLE_PERIOD_MS    100u

/* Status blink codes - see the header comment. */
#define BLINK_BOOT          1u
#define BLINK_IMU_READY     2u

/*********************************************************************
 * @fn      status_blink
 *
 * @brief   Blinks the indicator a given number of times, then pauses, so
 *          the count can be read off by eye.
 *
 * @param   count - number of blinks.
 *
 * @return  none
 */
static void status_blink(uint8_t count)
{
    uint8_t i;

    for(i = 0; i < count; i++)
    {
        LED_On();
        Delay_Ms(150);
        LED_Off();
        Delay_Ms(150);
    }

    Delay_Ms(400);
}

/*********************************************************************
 * @fn      fault_blink
 *
 * @brief   Continuous fast blink. Never returns: there is nothing useful
 *          to do with a sensor that will not answer.
 *
 * @return  does not return
 */
static void fault_blink(void)
{
    while(1)
    {
        LED_Toggle();
        Delay_Ms(100);
    }
}

/*********************************************************************
 * @fn      main
 *
 * @return  does not return
 */
int main(void)
{
    int16_t  gyro[3];
    int16_t  accel[3];
    uint8_t  whoami;
    uint8_t  src;
    uint32_t sample = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    /* Before any printf. On battery this is the only thing that can speak. */
    init_LED();
    status_blink(BLINK_BOOT);

    SDI_Printf_Enable();
    printf("\r\n=== LiveArrow baseline ===\r\n");
    printf("SystemClk : %u Hz\r\n", (unsigned int)SystemCoreClock);
    printf("ChipID    : %08x\r\n", (unsigned int)DBGMCU_GetCHIPID());
    printf("SDI host  : %s\r\n", SDI_Host_Present() ? "listening" : "absent");

    init_IMU();

    whoami = imu_who_am_i();
    printf("WHO_AM_I  : 0x%02x (expect 0x%02x)\r\n",
           whoami, IMU_WHO_AM_I_VALUE);

    if(whoami != IMU_WHO_AM_I_VALUE)
    {
        printf("Bus not working.\r\n");
        printf("  0x00 - MISO held low: CS not reaching the sensor, or it\r\n");
        printf("         has no power.\r\n");
        printf("  0xFF - MISO idle high: no reply at all; check SCK and\r\n");
        printf("         power.\r\n");
        fault_blink();                  /* does not return */
    }

    imu_config();
    status_blink(BLINK_IMU_READY);

    printf("Sensor configured. Reading every %u ms.\r\n",
           (unsigned int)SAMPLE_PERIOD_MS);
    printf("Raw counts. Accel +/-16 g: 1 g is about 2049, so at rest one\r\n");
    printf("axis sits near +/-2049 and the other two near 0.\r\n");
    printf("Gyro +/-4000 dps: about 7.14 counts per dps.\r\n\r\n");

    while(1)
    {
        /* The wake-up engine runs on its own and asserts INT1 independently
         * of this loop. Reading WAKE_UP_SRC is what releases the latched
         * pin; without it INT1 stays high and never fires again. */
        if(g_imu_int_flag != 0u)
        {
            g_imu_int_flag = 0;
            src = imu_clear_int();
            printf("  INT1  src=0x%02x\r\n", src);
            LED_Toggle();
        }

        imu_read_raw(gyro, accel);

        printf("[%5u] g=%6d %6d %6d   a=%6d %6d %6d\r\n",
               (unsigned int)sample,
               gyro[0],  gyro[1],  gyro[2],
               accel[0], accel[1], accel[2]);

        sample++;
        Delay_Ms(SAMPLE_PERIOD_MS);
    }
}
