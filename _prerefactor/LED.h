/*
 * LED.h - indicator module.
 *
 * Owns pin PD0 and timer TIM2. Nothing outside this module needs to know
 * that the indicator is active low, or that the brightness limiting and the
 * post-launch flash pattern are produced in software.
 */
#ifndef __LED_H
#define __LED_H

#include <stdint.h>

/* Configures PD0 and prepares TIM2. The indicator is left off and the
 * timer is left stopped, so an idle system takes no interrupts. */
void init_LED(void);

/* Direct control, at full brightness with no duty limiting. Each of these
 * cancels any running pattern and stops the timer, so the caller owns the
 * pin until a further call. Used by the fault indication and diagnostics. */
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

/* Start the post-launch indication sequence and return immediately:
 *
 *   phase 1   continuous for LED_SOLID_S seconds
 *   phase 2   LED_BLINK_ON_S seconds in every LED_BLINK_PERIOD_S seconds,
 *             repeating until LED_Off() or power-down
 *
 * Throughout both phases the indicator is driven at LED_DUTY_PCT duty at
 * LED_PWM_FREQ_HZ, so average current is reduced without visible flicker.
 * The sequence is produced by the TIM2 interrupt; the caller does not
 * block and does not need to service it. */
void LED_Trigger(void);

/* Non-zero while the LED_Trigger() sequence is running. */
uint8_t LED_PatternActive(void);

/* Called only from TIM2_IRQHandler. Declared here because the interrupt
 * vectors live together in ch32v00X_it.c by convention; it is not part of
 * the module's public interface and nothing else should call it. */
void LED_TimerISR(void);

#endif /* __LED_H */
