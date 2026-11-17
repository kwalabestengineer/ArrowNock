/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v00X_it.c
 * Description        : Interrupt service routines.
 *
 *                      One handler matters here: EXTI line 0, which carries
 *                      the IMU's INT1 on PC0.
 *******************************************************************************/
#include <ch32v00X_it.h>
#include "IMU.h"

void NMI_Handler(void)        __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   Non-maskable interrupt. Nothing on this board raises it.
 *
 * @return  none
 */
void NMI_Handler(void)
{
    while(1)
    {
    }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   Unrecoverable fault. Resetting is more useful in the field than
 *          sitting in a loop, and the cause is visible in the debugger
 *          either way.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while(1)
    {
    }
}

/*********************************************************************
 * @fn      EXTI7_0_IRQHandler
 *
 * @brief   EXTI lines 0 to 7 share one vector. Line 0 is PC0, the
 *          ISM330DHCX INT1 output.
 *
 *          This handler deliberately does nothing but raise a flag. The
 *          sensor's latched interrupt is released by reading WAKE_UP_SRC,
 *          and that read is a bus transaction - it belongs in the main
 *          loop, not inside an interrupt.
 *
 * @return  none
 */
void EXTI7_0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        g_imu_int_flag = 1;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
