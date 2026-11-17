/*
 * IMU.c - ISM330DHCX, register level, software SPI.
 *
 * Board wiring (U1 CH32V005D6U6 -> U2 ISM330DHCX):
 *   PA1  SCK   ->  U2 pin 13  SPC
 *   PC6  MOSI  ->  U2 pin 14  SDI
 *   PC7  MISO  <-  U2 pin  1  SDO
 *   PC4  CS    ->  U2 pin 12  CS
 *   PC0  INT1  <-  U2 pin  4  INT1      (EXTI line 0)
 *
 * The bus is driven by software, not by the SPI peripheral. The board puts
 * SCK on PA1 while the peripheral's SCK is PC5, and no alternate-function
 * mapping was found that reaches this pin set. Four plain GPIOs and a shift
 * loop need no mapping at all, which removes the question entirely.
 *
 * This file is the only place that knows any of the above.
 *
 * LAYOUT
 *   1. pin map and register addresses
 *   2. transport      - spi_xfer(), one byte in each direction
 *   3. register access - read, write, burst read
 *   4. bring-up       - GPIO and EXTI setup
 *   5. sensor API     - what IMU.h exposes
 */
#include "IMU.h"
#include <ch32v00X.h>
#include <debug.h>          /* Delay_Ms */

/* ========================================================================
 * 1. Pin map and registers
 * ======================================================================== */

#define IMU_CS_PORT         GPIOC
#define IMU_CS_PIN          GPIO_Pin_4
#define IMU_SCK_PORT        GPIOA
#define IMU_SCK_PIN         GPIO_Pin_1
#define IMU_MOSI_PORT       GPIOC
#define IMU_MOSI_PIN        GPIO_Pin_6
#define IMU_MISO_PORT       GPIOC
#define IMU_MISO_PIN        GPIO_Pin_7
#define IMU_INT_PORT        GPIOC
#define IMU_INT_PIN         GPIO_Pin_0

#define CS_LOW()            GPIO_ResetBits(IMU_CS_PORT,   IMU_CS_PIN)
#define CS_HIGH()           GPIO_SetBits(IMU_CS_PORT,     IMU_CS_PIN)
#define SCK_LOW()           GPIO_ResetBits(IMU_SCK_PORT,  IMU_SCK_PIN)
#define SCK_HIGH()          GPIO_SetBits(IMU_SCK_PORT,    IMU_SCK_PIN)
#define MOSI_LOW()          GPIO_ResetBits(IMU_MOSI_PORT, IMU_MOSI_PIN)
#define MOSI_HIGH()         GPIO_SetBits(IMU_MOSI_PORT,   IMU_MOSI_PIN)
#define MISO_READ()         GPIO_ReadInputDataBit(IMU_MISO_PORT, IMU_MISO_PIN)

/* ISM330DHCX registers, datasheet section 9. Only the ones used here. */
#define REG_WHO_AM_I        0x0F    /* -> 0x6B                              */
#define REG_CTRL1_XL        0x10    /* accel ODR[7:4], FS[3:2]              */
#define REG_CTRL2_G         0x11    /* gyro  ODR[7:4], FS[3:2], FS_4000[0]  */
#define REG_CTRL3_C         0x12    /* BOOT7 BDU6 ... IF_INC2 SW_RESET0     */
#define REG_CTRL4_C         0x13    /* I2C_disable = bit 2                  */
#define REG_WAKE_UP_SRC     0x1B    /* FF_IA5 SLEEP4 WU_IA3 X2 Y1 Z0        */
#define REG_OUTX_L_G        0x22    /* 6 gyro then 6 accel, auto-increment  */
#define REG_TAP_CFG0        0x56    /* SLOPE_FDS4 ... LIR0                  */
#define REG_TAP_CFG2        0x58    /* INTERRUPTS_ENABLE = bit 7            */
#define REG_WAKE_UP_THS     0x5B    /* WK_THS[5:0]                          */
#define REG_WAKE_UP_DUR     0x5C    /* FF_DUR5[7] WAKE_DUR[6:5] THS_W[4]    */
#define REG_MD1_CFG         0x5E    /* INT1_WU = bit 5                      */

/* Wake-up threshold, in steps of full-scale/64 = 0.25 g at +/-16 g.
 * The field is 6 bits, so 63 (15.75 g) is the ceiling.
 *
 * 8 => 2 g, which a firm tap on the bench will trip. That makes the
 * interrupt path testable by hand. Raise it toward the ceiling when the
 * sensor is in an arrow and only a launch should wake it. */
#define IMU_WAKE_THS_STEPS  8u

volatile uint8_t g_imu_int_flag = 0;

/* ========================================================================
 * 2. Transport
 * ======================================================================== */

/* Half a clock period. At 48 MHz and -Os this lands somewhere near 1 MHz,
 * well inside the sensor's 10 MHz ceiling. Raise the count to slow the bus
 * down while probing it. */
static void sck_delay(void)
{
    volatile uint8_t d = 12;
    while(d != 0u) { d--; }
}

/* SPI mode 3 in software: the clock idles HIGH, data is presented on the
 * falling edge and sampled on the rising edge, most significant bit first.
 * Sends one byte and returns the byte shifted back at the same time. */
static uint8_t spi_xfer(uint8_t out)
{
    uint8_t in = 0;
    uint8_t i;

    for(i = 0; i < 8u; i++)
    {
        /* falling edge - present the next bit */
        SCK_LOW();
        if((out & 0x80u) != 0u) { MOSI_HIGH(); } else { MOSI_LOW(); }
        out <<= 1;
        sck_delay();

        /* rising edge - the sensor's bit is valid here */
        SCK_HIGH();
        in <<= 1;
        if(MISO_READ() != 0u) { in |= 1u; }
        sck_delay();
    }

    return in;
}

/* ========================================================================
 * 3. Register access
 *
 * Bit 7 of the address byte selects direction: clear to write, set to read.
 * ======================================================================== */

static void imu_write_reg(uint8_t reg, uint8_t val)
{
    CS_LOW();
    spi_xfer(reg & 0x7Fu);
    spi_xfer(val);
    CS_HIGH();
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t val;

    CS_LOW();
    spi_xfer(reg | 0x80u);
    val = spi_xfer(0xFFu);          /* dummy out, data in */
    CS_HIGH();

    return val;
}

/* CS stays low for the whole transfer. That is what lets the sensor's
 * address auto-increment walk through consecutive registers. */
static void imu_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    CS_LOW();
    spi_xfer(reg | 0x80u);
    while(len-- != 0u)
    {
        *buf++ = spi_xfer(0xFFu);
    }
    CS_HIGH();
}

/* ========================================================================
 * 4. Bring-up
 * ======================================================================== */

static void imu_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOC |
                          RCC_PB2Periph_AFIO, ENABLE);

    /* PA1 and PA2 are the oscillator pins. Until this remap is enabled PA1
     * is not usable as a pin at all, so SCK would never move. Safe here:
     * the board runs from the internal HSI and carries no crystal. */
    GPIO_PinRemapConfig(GPIO_Remap_PA1_2, ENABLE);

    /* Idle states BEFORE the pins become outputs: CS deasserted, and SCK
     * high because mode 3 idles high. Setting them afterwards would put a
     * spurious edge on the bus at every reset. */
    GPIO_SetBits(IMU_CS_PORT,  IMU_CS_PIN);
    GPIO_SetBits(IMU_SCK_PORT, IMU_SCK_PIN);

    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;

    g.GPIO_Pin = IMU_CS_PIN;
    GPIO_Init(IMU_CS_PORT, &g);

    g.GPIO_Pin = IMU_SCK_PIN;
    GPIO_Init(IMU_SCK_PORT, &g);

    g.GPIO_Pin = IMU_MOSI_PIN;
    GPIO_Init(IMU_MOSI_PORT, &g);

    /* Pull-up rather than floating. If the sensor is absent or unpowered
     * this reads back 0xFF, which says "no reply", instead of a floating
     * 0x00 that looks like a real answer. */
    g.GPIO_Pin  = IMU_MISO_PIN;
    g.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(IMU_MISO_PORT, &g);
}

static void imu_exti_init(void)
{
    GPIO_InitTypeDef g = {0};
    EXTI_InitTypeDef e = {0};
    NVIC_InitTypeDef n = {0};

    /* INT1 idles low - CTRL3_C H_LACTIVE defaults to active high - so a
     * pull-down matches it and the event is a rising edge. */
    g.GPIO_Pin   = IMU_INT_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPD;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(IMU_INT_PORT, &g);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);

    e.EXTI_Line    = EXTI_Line0;
    e.EXTI_Mode    = EXTI_Mode_Interrupt;
    e.EXTI_Trigger = EXTI_Trigger_Rising;
    e.EXTI_LineCmd = ENABLE;
    EXTI_Init(&e);

    n.NVIC_IRQChannel                   = EXTI7_0_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority        = 1;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);
}

void init_IMU(void)
{
    imu_gpio_init();
    imu_exti_init();

    /* The sensor needs 10-20 ms after power-up before it answers. Reading
     * it during its own boot returns garbage. */
    Delay_Ms(20);
}

/* ========================================================================
 * 5. Sensor API
 * ======================================================================== */

uint8_t imu_who_am_i(void)
{
    return imu_read_reg(REG_WHO_AM_I);
}

void imu_config(void)
{
    uint32_t guard;

    /* Software reset, then wait for the bit to self-clear. Bounded, so a
     * dead bus falls through instead of trapping here forever. */
    imu_write_reg(REG_CTRL3_C, 0x01);
    guard = 100000u;
    while((imu_read_reg(REG_CTRL3_C) & 0x01u) != 0u)
    {
        if(--guard == 0u) { break; }
    }
    Delay_Ms(20);                       /* let the boot sequence finish */

    imu_write_reg(REG_CTRL4_C, 0x04);   /* I2C_disable - SPI only       */
    imu_write_reg(REG_CTRL3_C, 0x44);   /* BDU | IF_INC                 */

    /* Both sensors at maximum full scale and 208 Hz.
     *   accel  0x05 << 4 = 208 Hz, 0x01 << 2 = +/-16 g
     *   gyro   0x05 << 4 = 208 Hz, bit 0 FS_4000 = +/-4000 dps          */
    imu_write_reg(REG_CTRL1_XL, (0x05u << 4) | (0x01u << 2));
    imu_write_reg(REG_CTRL2_G,  (0x05u << 4) | 0x01u);

    /* Wake-up engine.
     * SLOPE_FDS (bit 4) runs it on high-pass filtered data, so gravity is
     * removed and a stationary board at 1 g cannot trip it.
     * LIR (bit 0) latches INT1 until WAKE_UP_SRC is read - see
     * imu_clear_int(). */
    imu_write_reg(REG_TAP_CFG0, 0x11);

    /* WAKE_DUR = 1 in bits 6:5: the threshold must be exceeded on two
     * consecutive samples rather than one, so a single-sample electrical
     * glitch no longer trips it. THS_W stays 0, which puts WK_THS in
     * full-scale/64 steps. */
    imu_write_reg(REG_WAKE_UP_DUR, 0x20);
    imu_write_reg(REG_WAKE_UP_THS, (uint8_t)(IMU_WAKE_THS_STEPS & 0x3Fu));

    /* Settle before arming.
     *
     * The high-pass filter's reference starts at zero, so bringing the
     * accelerometer out of power-down looks to it like a step of the full
     * standing 1 g, and it overshoots for the first samples. Arming the
     * interrupt during that window latches a wake-up on every axis holding
     * a DC component - which is why a motionless board used to report
     * WAKE_UP_SRC = 0x0d, with X and Z flagged and Y, sitting near zero,
     * quiet.
     *
     * So: let it converge, discard whatever latched, and only then route
     * the event to the pin. */
    Delay_Ms(150);
    (void)imu_read_reg(REG_WAKE_UP_SRC);

    imu_write_reg(REG_MD1_CFG,  0x20);  /* INT1_WU -> INT1 pin          */
    imu_write_reg(REG_TAP_CFG2, 0x80);  /* master enable, deliberately last */

    /* INT1 may have glitched while the routing bits were being written,
     * and EXTI has been live since init_IMU(). */
    Delay_Ms(20);
    (void)imu_read_reg(REG_WAKE_UP_SRC);
    g_imu_int_flag = 0;
}

void imu_read_raw(int16_t gyro[3], int16_t accel[3])
{
    uint8_t raw[12];

    imu_read_burst(REG_OUTX_L_G, raw, 12);

    if(gyro != 0)
    {
        gyro[0]  = (int16_t)((uint16_t)raw[0]  | ((uint16_t)raw[1]  << 8));
        gyro[1]  = (int16_t)((uint16_t)raw[2]  | ((uint16_t)raw[3]  << 8));
        gyro[2]  = (int16_t)((uint16_t)raw[4]  | ((uint16_t)raw[5]  << 8));
    }
    if(accel != 0)
    {
        accel[0] = (int16_t)((uint16_t)raw[6]  | ((uint16_t)raw[7]  << 8));
        accel[1] = (int16_t)((uint16_t)raw[8]  | ((uint16_t)raw[9]  << 8));
        accel[2] = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));
    }
}

uint8_t imu_clear_int(void)
{
    return imu_read_reg(REG_WAKE_UP_SRC);
}
