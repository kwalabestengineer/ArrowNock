/*
 * LED.h - indicator on PD0.
 *
 * The pin number and the fact that the drive is active low are private to
 * LED.c. Nothing outside this module needs to know either.
 */
#ifndef __LED_H
#define __LED_H

#include <stdint.h>

/* Configures PD0 as an output and leaves the indicator off.
 * Call this first in main() - before any printf - so that a board with no
 * debugger attached still has a way to show what it is doing. */
void init_LED(void);

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

/* Non-zero while the indicator is lit. */
uint8_t LED_IsOn(void);

#endif /* __LED_H */
