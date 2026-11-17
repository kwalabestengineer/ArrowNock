/*
 * IMU.h - ISM330DHCX 6-axis IMU.
 *
 * The transport (hardware SPI1 vs software bit-bang), the pin map and the
 * register addresses all stay private to IMU.c.
 */
#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

/* Set by EXTI7_0_IRQHandler when the sensor asserts INT1 (PC0).
 * Cleared by the main loop. Must be volatile: the ISR writes it behind the
 * compiler's back, and at -Os a plain variable would be cached in a register
 * so that while(!flag) never exits. */
extern volatile uint8_t g_imu_int_flag;

/* Hardware bring-up only: SCK/MOSI/MISO, CS, and INT1 -> EXTI0.
 * Does not talk to the sensor, so it is safe to call before the bus works. */
void init_IMU(void);

/* WHO_AM_I (0x0F). Expect 0x6B. */
uint8_t imu_who_am_i(void);

/* Software reset, then accel/gyro full scale + ODR, then the wake-up
 * threshold engine routed to INT1. Only call once imu_who_am_i() == 0x6B. */
void imu_config(void);

/* One burst from OUTX_L_G: 6 gyro bytes then 6 accel bytes.
 * Raw LSB counts - no float conversion (this part has no FPU).
 * Pass NULL for either array if you don't need it. */
void imu_read_raw(int16_t gyro[3], int16_t accel[3]);

/* Read WAKE_UP_SRC (0x1B). This is what clears the latched INT1, so the
 * handler path must call it or the pin stays high and you get no more
 * interrupts. Returns the register: bit3 = WU_IA, bit5 = FF_IA. */
uint8_t imu_clear_int(void);

/* ---- diagnostics ---- */

/* Slowly toggles SCK, MOSI and CS one at a time, ~2 s each, so you can put a
 * multimeter or LED on the pads and confirm they actually move. Prints what
 * it is doing. This is the test for "is GPIO_Remap_PA1_2 working". */
void imu_pin_test(void);

/* Sweeps every SPI1 remap option and reports which one makes WHO_AM_I answer
 * 0x6B. Only meaningful when IMU_USE_BITBANG is 0. Returns the working
 * constant, or 0 if none worked. */
uint32_t imu_find_remap(void);

#endif /* __IMU_H */
