/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v00X_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/11/04
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include <ch32v00X_it.h>
#include "IMU.h"
#include "LED.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  NVIC_SystemReset();
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      EXTI7_0_IRQHandler
 *
 * @brief   EXTI lines 0..7 share one vector. Line 0 is PC0 = ISM330DHCX INT1.
 *          Kept to a flag on purpose: the SPI read that clears the sensor's
 *          latched interrupt (WAKE_UP_SRC) belongs in the main loop, not in
 *          an ISR.
 *
 * @return  none
 */
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        g_imu_int_flag = 1;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   Time base for the indicator: duty limiting and the post-launch
 *          flash pattern. Runs only while a pattern is active - the timer
 *          is stopped otherwise, so an idle system takes no interrupts and
 *          the processor can stay halted between launches.
 *
 * @return  none
 */
void TIM2_IRQHandler(void)
{
    LED_TimerISR();
}
