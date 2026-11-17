/*
 * IMU.h - ISM330DHCX 6-axis inertial sensor.
 *
 * The pin map, the software SPI transport and the register addresses all
 * stay private to IMU.c.
 */
#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

#define IMU_WHO_AM_I_VALUE  0x6B

/* Set by EXTI7_0_IRQHandler when the sensor asserts INT1 on PC0; cleared by
 * the main loop. Must be volatile: the handler writes it behind the
 * compiler's back, and at -Os a plain variable would be cached in a register
 * so that while(!flag) would never exit. */
extern volatile uint8_t g_imu_int_flag;

/* Pin and interrupt bring-up only: the four bus pins, plus INT1 -> EXTI
 * line 0. Does not talk to the sensor, so it is safe to call before the bus
 * is known to work. */
void init_IMU(void);

/* WHO_AM_I (0x0F). Returns IMU_WHO_AM_I_VALUE on a working bus.
 * 0x00 means MISO is held low - CS not reaching the sensor, or no power.
 * 0xFF means MISO is idling high - no reply at all; check SCK and power. */
uint8_t imu_who_am_i(void);

/* Software reset, then full scale and output data rate on both sensors,
 * then the wake-up threshold engine routed to INT1.
 * Only call once imu_who_am_i() has returned IMU_WHO_AM_I_VALUE. */
void imu_config(void);

/* One burst read from OUTX_L_G: six gyro bytes then six accelerometer bytes,
 * using the sensor's address auto-increment. Raw LSB counts, no unit
 * conversion - this part has no FPU. Pass NULL for either array.
 *
 * Scale at the settings imu_config() applies:
 *   accel  +/-16 g     0.488 mg per count   (1 g is about 2049)
 *   gyro   +/-4000 dps  140 mdps per count  (1 dps is about 7.14)
 */
void imu_read_raw(int16_t gyro[3], int16_t accel[3]);

/* Reads WAKE_UP_SRC (0x1B), which is what releases the latched INT1 line.
 * The interrupt path must call this or the pin stays high and no further
 * interrupt is ever delivered.
 * Returns the register: bit 3 = WU_IA, bits 2..0 = which axis. */
uint8_t imu_clear_int(void);

#endif /* __IMU_H */
