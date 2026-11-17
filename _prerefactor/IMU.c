/*
 * IMU.c - ISM330DHCX, register-level driver.
 *
 * Board wiring (from the schematic):
 *   PA1  SCK   ->  U2 pin 13 SPC
 *   PC6  MOSI  ->  U2 pin 14 SDI
 *   PC7  MISO  <-  U2 pin  1 SDO
 *   PC4  CS    ->  U2 pin 12 CS
 *   PC0  INT1  <-  U2 pin  4 INT1   (EXTI line 0)
 *
 * Two transports, chosen at compile time:
 *
 *   IMU_USE_BITBANG 1  - all four pins are plain GPIO and the byte transfer
 *                        is a software loop. Needs NO alternate-function
 *                        mapping, so it sidesteps the SPI1 remap question
 *                        entirely. Use this to prove the hardware.
 *
 *   IMU_USE_BITBANG 0  - the SPI1 peripheral, which needs IMU_SPI1_REMAP
 *                        to be the correct value from the reference manual.
 *
 * Everything above spi_xfer() is identical either way.
 */
#include "IMU.h"
#include "debug.h"          /* pulls in ch32v00X.h, printf, Delay_Ms */

/* ======================================================================== */
#define IMU_USE_BITBANG     1
/* ======================================================================== */

/* ------------------------------------------------------------------------
 * Pin map - private to this module on purpose.
 * ------------------------------------------------------------------------ */
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

#define CS_LOW()            GPIO_ResetBits(IMU_CS_PORT, IMU_CS_PIN)
#define CS_HIGH()           GPIO_SetBits(IMU_CS_PORT, IMU_CS_PIN)

#define SCK_LOW()           GPIO_ResetBits(IMU_SCK_PORT, IMU_SCK_PIN)
#define SCK_HIGH()          GPIO_SetBits(IMU_SCK_PORT, IMU_SCK_PIN)
#define MOSI_LOW()          GPIO_ResetBits(IMU_MOSI_PORT, IMU_MOSI_PIN)
#define MOSI_HIGH()         GPIO_SetBits(IMU_MOSI_PORT, IMU_MOSI_PIN)
#define MISO_READ()         GPIO_ReadInputDataBit(IMU_MISO_PORT, IMU_MISO_PIN)

/*  Only used when IMU_USE_BITBANG is 0.
 *  Default SPI1 SCK on this part is PC5; this board uses PA1 with MOSI/MISO
 *  still on PC6/PC7. The mapping table is in the CH32V00X reference manual,
 *  AFIO chapter, PCFR1, SPI1_RM field. Still unverified. */
#define IMU_SPI1_REMAP      GPIO_PartialRemap1_SPI1

/* Bounded spin so a dead peripheral reports failure instead of freezing. */
#define SPI_SPIN_LIMIT      20000u

/* ------------------------------------------------------------------------
 * ISM330DHCX registers (datasheet section 9)
 * ------------------------------------------------------------------------ */
#define REG_WHO_AM_I        0x0F    /* -> 0x6B                              */
#define REG_CTRL1_XL        0x10    /* accel ODR[7:4], FS[3:2]              */
#define REG_CTRL2_G         0x11    /* gyro  ODR[7:4], FS[3:2], FS_4000[0]  */
#define REG_CTRL3_C         0x12    /* BOOT7 BDU6 H_LACTIVE5 IF_INC2 SW_RST0*/
#define REG_CTRL4_C         0x13    /* I2C_disable = bit 2                  */
#define REG_WAKE_UP_SRC     0x1B    /* FF_IA5 SLEEP4 WU_IA3 X2 Y1 Z0        */
#define REG_STATUS_REG      0x1E    /* GDA1 XLDA0                           */
#define REG_OUTX_L_G        0x22    /* 6 gyro then 6 accel, auto-increment  */
#define REG_TAP_CFG0        0x56    /* SLOPE_FDS4 ... LIR0                  */
#define REG_TAP_CFG2        0x58    /* INTERRUPTS_ENABLE = bit 7            */
#define REG_WAKE_UP_THS     0x5B    /* WK_THS[5:0]                          */
#define REG_WAKE_UP_DUR     0x5C    /* FF_DUR5[7] WAKE_DUR[6:5] THS_W[4]    */
#define REG_FREE_FALL       0x5D    /* FF_DUR[7:3] FF_THS[2:0]              */
#define REG_MD1_CFG         0x5E    /* INT1_WU = bit 5, INT1_FF = bit 4     */

#define IMU_WHO_AM_I_VALUE  0x6B

volatile uint8_t g_imu_int_flag = 0;

/* ========================================================================
 * Transport
 * ======================================================================== */
#if IMU_USE_BITBANG

/* Half clock period. 48 MHz core, -Os: this lands somewhere around 1 MHz,
 * comfortably inside the sensor's 10 MHz ceiling. Raise the count if you
 * want to slow it down further while debugging. */
static void sck_delay(void)
{
    volatile uint8_t d = 12;
    while (d != 0u) { d--; }
}

/* SPI mode 3 in software: clock idles HIGH, data is presented on the
 * falling edge and sampled on the rising edge. MSB first. */
static uint8_t spi_xfer(uint8_t out)
{
    uint8_t in = 0;
    uint8_t i;

    for (i = 0; i < 8u; i++)
    {
        /* falling edge - present the next bit */
        SCK_LOW();
        if ((out & 0x80u) != 0u) { MOSI_HIGH(); } else { MOSI_LOW(); }
        out <<= 1;
        sck_delay();

        /* rising edge - sensor's bit is valid here */
        SCK_HIGH();
        in <<= 1;
        if (MISO_READ() != 0u) { in |= 1u; }
        sck_delay();
    }
    return in;
}

static void spi_wait_idle(void)
{
    /* Nothing to drain - the loop above finished the last edge itself. */
}

#else   /* hardware SPI1 */

static uint8_t spi_xfer(uint8_t out)
{
    uint32_t guard;

    guard = SPI_SPIN_LIMIT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (--guard == 0u) return 0xFFu;    /* bus dead - don't hang */
    }
    SPI_I2S_SendData(SPI1, out);

    guard = SPI_SPIN_LIMIT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (--guard == 0u) return 0xFFu;
    }
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* RXNE says the byte arrived, but the shift register may still be finishing
 * the last clock edge. Dropping CS early truncates it. */
static void spi_wait_idle(void)
{
    uint32_t guard = SPI_SPIN_LIMIT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)
    {
        if (--guard == 0u) return;
    }
}

#endif  /* IMU_USE_BITBANG */

/* ========================================================================
 * Register access - identical for both transports
 * ======================================================================== */

static void imu_write_reg(uint8_t reg, uint8_t val)
{
    CS_LOW();
    spi_xfer(reg & 0x7Fu);              /* bit 7 clear = write */
    spi_xfer(val);
    spi_wait_idle();
    CS_HIGH();
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t val;

    CS_LOW();
    spi_xfer(reg | 0x80u);              /* bit 7 set = read */
    val = spi_xfer(0xFFu);              /* dummy out, data in */
    spi_wait_idle();
    CS_HIGH();
    return val;
}

/* CS stays low for the whole transfer - that is what lets the sensor's
 * address auto-increment walk through consecutive registers. */
static void imu_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    CS_LOW();
    spi_xfer(reg | 0x80u);
    while (len--)
    {
        *buf++ = spi_xfer(0xFFu);
    }
    spi_wait_idle();
    CS_HIGH();
}

/* ========================================================================
 * Bring-up
 * ======================================================================== */

static void imu_gpio_init(uint32_t spi_remap)
{
    GPIO_InitTypeDef g = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOC |
                          RCC_PB2Periph_AFIO, ENABLE);

    /* PA1/PA2 are the oscillator pins (XI/XO). Until this remap is enabled
     * PA1 is not usable as a pin at all - which is true for bit-bang too,
     * so this line is required either way. Safe here: we run from the
     * internal HSI, there is no crystal on the board. */
    GPIO_PinRemapConfig(GPIO_Remap_PA1_2, ENABLE);

    /* Idle states BEFORE the pins become outputs:
     * CS high (deasserted) and SCK high (mode 3 idles high). */
    GPIO_SetBits(IMU_CS_PORT,  IMU_CS_PIN);
    GPIO_SetBits(IMU_SCK_PORT, IMU_SCK_PIN);

    g.GPIO_Pin   = IMU_CS_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(IMU_CS_PORT, &g);

#if IMU_USE_BITBANG
    (void)spi_remap;    /* no alternate function involved */

    g.GPIO_Pin  = IMU_SCK_PIN;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(IMU_SCK_PORT, &g);

    g.GPIO_Pin  = IMU_MOSI_PIN;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(IMU_MOSI_PORT, &g);

    /* Pull-up rather than floating: if the sensor is absent or unpowered
     * this reads 0xFF instead of a misleading 0x00. */
    g.GPIO_Pin  = IMU_MISO_PIN;
    g.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(IMU_MISO_PORT, &g);
#else
    if (spi_remap != 0u)
    {
        GPIO_PinRemapConfig(spi_remap, ENABLE);
    }

    g.GPIO_Pin  = IMU_SCK_PIN;
    g.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(IMU_SCK_PORT, &g);

    g.GPIO_Pin  = IMU_MOSI_PIN;
    g.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(IMU_MOSI_PORT, &g);

    g.GPIO_Pin  = IMU_MISO_PIN;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(IMU_MISO_PORT, &g);
#endif
}

#if !IMU_USE_BITBANG
static void imu_spi_init(void)
{
    SPI_InitTypeDef s = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_SPI1, ENABLE);
    SPI_Cmd(SPI1, DISABLE);

    s.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    s.SPI_Mode              = SPI_Mode_Master;
    s.SPI_DataSize          = SPI_DataSize_8b;
    s.SPI_CPOL              = SPI_CPOL_High;    /* mode 3: clock idles high */
    s.SPI_CPHA              = SPI_CPHA_2Edge;   /* mode 3: sample 2nd edge  */
    s.SPI_NSS               = SPI_NSS_Soft;     /* we drive PC4 ourselves   */
    s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  /* 48M/128 = 375k */
    s.SPI_FirstBit          = SPI_FirstBit_MSB;
    s.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &s);

    /* SPI_NSS_Soft only sets SSM. SSI still reads 0, which the master takes
     * as "another master pulled NSS low" -> mode fault, and the peripheral
     * quietly disables itself. Forcing SSI high is not optional. */
    SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

    SPI_Cmd(SPI1, ENABLE);
}
#endif

static void imu_exti_init(void)
{
    GPIO_InitTypeDef g = {0};
    EXTI_InitTypeDef e = {0};
    NVIC_InitTypeDef n = {0};

    /* INT1 idles low (CTRL3_C H_LACTIVE defaults to active-high), so a
     * pull-down matches and a rising edge is the event. */
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
    imu_gpio_init(IMU_SPI1_REMAP);
#if !IMU_USE_BITBANG
    imu_spi_init();
#endif
    imu_exti_init();

#if IMU_USE_BITBANG
    printf("IMU transport: BIT-BANG (GPIO, no remap needed)\r\n");
#else
    printf("IMU transport: hardware SPI1, remap 0x%08x\r\n",
           (unsigned int)IMU_SPI1_REMAP);
#endif

    /* The sensor needs ~10-20 ms after power-up before it answers.
     * Talking to it during boot returns garbage on the first read. */
    Delay_Ms(20);
}

/* ========================================================================
 * Sensor API
 * ======================================================================== */

uint8_t imu_who_am_i(void)
{
    return imu_read_reg(REG_WHO_AM_I);
}

void imu_config(void)
{
    uint32_t guard;

    /* --- software reset, then wait for it to self-clear --- */
    imu_write_reg(REG_CTRL3_C, 0x01);
    guard = 100000u;
    while ((imu_read_reg(REG_CTRL3_C) & 0x01u) != 0u)
    {
        if (--guard == 0u) break;       /* bounded: never trap here */
    }
    Delay_Ms(20);                       /* let the boot sequence finish     */

    imu_write_reg(REG_CTRL4_C, 0x04);   /* I2C_disable - we are SPI only    */
    imu_write_reg(REG_CTRL3_C, 0x44);   /* BDU | IF_INC                     */

    /* Full scale at maximum on both: launch is ~690 g and fletching spin
     * can exceed 4000 dps, so we want to saturate as late as possible.
     * 208 Hz because the launch impulse is only ~20 ms - at 26 Hz we would
     * sample every 38 ms and could miss it entirely. */
    imu_write_reg(REG_CTRL1_XL, (0x05u << 4) | (0x01u << 2)); /* 208Hz, 16g  */
    imu_write_reg(REG_CTRL2_G,  (0x05u << 4) | 0x01u);        /* 208Hz, 4000 */

    /* --- wake-up threshold engine --- */
    /* SLOPE_FDS puts the wake-up function on high-pass filtered data so
     * gravity is removed and a stationary arrow at 1 g cannot trip it.
     * LIR latches INT1 until WAKE_UP_SRC is read - see imu_clear_int(). */
    imu_write_reg(REG_TAP_CFG0,    0x11);

    /* WAKE_DUR = 1 (bits 6:5): the threshold must be exceeded on TWO
     * consecutive samples, not one. A launch is 13-20 ms - three or four
     * samples at 208 Hz - so this costs nothing, while a single-sample
     * electrical glitch or a sharp tick no longer trips the trigger.
     * WAKE_THS_W stays 0, so WK_THS is in FS/64 = 0.25 g steps. */
    imu_write_reg(REG_WAKE_UP_DUR, 0x20);

    /* 63 x 0.25 g = 15.75 g, the maximum this 6-bit field can express.
     * A shot is 300-700 g, so nothing is lost by sitting at the ceiling -
     * and with launch-only detection (BALLISTIC_SAMPLES 0 in main.c) this
     * threshold is the ONLY gate, so it should be as high as it goes. */
    imu_write_reg(REG_WAKE_UP_THS, 63);

    /* --- settle before arming ---------------------------------------
     * SLOPE_FDS runs the wake-up function through a high-pass filter, and
     * that filter's reference starts at zero. Bringing the accelerometer
     * out of power-down therefore looks like a step of the full standing
     * 1 g, and the filter overshoots hard for the first samples. Arm the
     * interrupt during that window and you latch a wake-up on every axis
     * that carries a DC component - which is why a motionless board
     * reported WAKE_UP_SRC = 0x0d (X_WU | Z_WU | WU_IA) while Y, sitting
     * at zero, stayed quiet.
     *
     * So: let it converge, throw away anything latched, and only then
     * route the event to the pin. */
    Delay_Ms(150);
    (void)imu_read_reg(REG_WAKE_UP_SRC);  /* discard the settling transient  */

    imu_write_reg(REG_MD1_CFG,     0x20); /* INT1_WU -> INT1 pin              */

    /* Master enable for the embedded interrupt functions. Deliberately last,
     * so nothing latches a spurious event mid-configuration. */
    imu_write_reg(REG_TAP_CFG2,    0x80);

    /* Belt and braces: INT1 may have glitched while the routing bits were
     * being written, and EXTI has been live since init_IMU(). */
    Delay_Ms(20);
    (void)imu_read_reg(REG_WAKE_UP_SRC);
    g_imu_int_flag = 0;
}

void imu_read_raw(int16_t gyro[3], int16_t accel[3])
{
    uint8_t raw[12];

    imu_read_burst(REG_OUTX_L_G, raw, 12);

    if (gyro != 0)
    {
        gyro[0]  = (int16_t)((uint16_t)raw[0]  | ((uint16_t)raw[1]  << 8));
        gyro[1]  = (int16_t)((uint16_t)raw[2]  | ((uint16_t)raw[3]  << 8));
        gyro[2]  = (int16_t)((uint16_t)raw[4]  | ((uint16_t)raw[5]  << 8));
    }
    if (accel != 0)
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

/* ========================================================================
 * Diagnostics
 * ======================================================================== */

/* Put a multimeter (or an LED through a 1k resistor to GND) on each pad in
 * turn. A pin that never moves is not connected to the core - for PA1 that
 * means GPIO_Remap_PA1_2 did not take effect, and no SPI configuration on
 * earth will make SCK appear there. */
void imu_pin_test(void)
{
    uint8_t i;

    printf("--- pin test: 5 slow pulses each, ~1 s per level ---\r\n");

    printf("  SCK  (PA1)\r\n");
    for (i = 0; i < 5u; i++) { SCK_HIGH();  Delay_Ms(1000);
                               SCK_LOW();   Delay_Ms(1000); }
    SCK_HIGH();

    printf("  MOSI (PC6)\r\n");
    for (i = 0; i < 5u; i++) { MOSI_HIGH(); Delay_Ms(1000);
                               MOSI_LOW();  Delay_Ms(1000); }

    printf("  CS   (PC4)\r\n");
    for (i = 0; i < 5u; i++) { CS_HIGH();   Delay_Ms(1000);
                               CS_LOW();    Delay_Ms(1000); }
    CS_HIGH();

    printf("  MISO (PC7) reads: %d  (idle, pulled up -> expect 1)\r\n",
           (int)MISO_READ());
    printf("--- pin test done ---\r\n");
}

uint32_t imu_find_remap(void)
{
#if IMU_USE_BITBANG
    printf("--- remap sweep skipped: bit-bang transport uses no remap ---\r\n");
    return 0u;
#else
    static const uint32_t options[] = {
        0u,                             /* no remap (SCK would be PC5)      */
        GPIO_PartialRemap1_SPI1,
        GPIO_PartialRemap2_SPI1,
        GPIO_PartialRemap3_SPI1,
        GPIO_PartialRemap4_SPI1,
        GPIO_PartialRemap5_SPI1,
        GPIO_FullRemap_SPI1
    };
    uint8_t i;
    uint8_t id;

    printf("--- SPI1 remap sweep ---\r\n");

    for (i = 0; i < (sizeof(options) / sizeof(options[0])); i++)
    {
        GPIO_PinRemapConfig(GPIO_PartialRemap1_SPI1, DISABLE);
        GPIO_PinRemapConfig(GPIO_FullRemap_SPI1,     DISABLE);

        imu_gpio_init(options[i]);
        imu_spi_init();
        Delay_Ms(5);

        id = imu_who_am_i();
        printf("  option %d (0x%08x): WHO_AM_I = 0x%02x%s\r\n",
               i, (unsigned int)options[i], id,
               (id == IMU_WHO_AM_I_VALUE) ? "   <== THIS ONE" : "");

        if (id == IMU_WHO_AM_I_VALUE)
        {
            printf("--- set IMU_SPI1_REMAP to option %d ---\r\n", i);
            return options[i];
        }
    }

    printf("--- no remap worked: check CS, ground, power, MISO ---\r\n");
    return 0u;
#endif
}
