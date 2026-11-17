/*
 * LED.c - indicator on PD0.
 *
 * Board wiring: D1/D2 anodes sit on +3V3 through 330R series resistors, so
 * the pin SINKS the current. Driving PD0 LOW lights the indicator. That
 * inversion is confined to this file - LED_On() means lit, everywhere else.
 */
#include "LED.h"
#include <ch32v00X.h>

#define LED_PORT        GPIOD
#define LED_PIN         GPIO_Pin_0
#define LED_RCC         RCC_PB2Periph_GPIOD

#define LED_DRIVE_ON()  GPIO_ResetBits(LED_PORT, LED_PIN)   /* active low */
#define LED_DRIVE_OFF() GPIO_SetBits(LED_PORT, LED_PIN)

void init_LED(void)
{
    GPIO_InitTypeDef g = {0};

    RCC_PB2PeriphClockCmd(LED_RCC, ENABLE);

    /* Preload the output register HIGH before the pin becomes an output.
     * It resets to zero, so configuring the pin first would drive it low
     * and flash the indicator at every reset. */
    GPIO_SetBits(LED_PORT, LED_PIN);

    g.GPIO_Pin   = LED_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(LED_PORT, &g);

    LED_DRIVE_OFF();
}

void LED_On(void)
{
    LED_DRIVE_ON();
}

void LED_Off(void)
{
    LED_DRIVE_OFF();
}

void LED_Toggle(void)
{
    if(GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) == Bit_RESET)
    {
        LED_DRIVE_OFF();        /* pin low  => lit  => extinguish */
    }
    else
    {
        LED_DRIVE_ON();         /* pin high => dark => light      */
    }
}

uint8_t LED_IsOn(void)
{
    return (uint8_t)((GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) == Bit_RESET)
                     ? 1u : 0u);
}
