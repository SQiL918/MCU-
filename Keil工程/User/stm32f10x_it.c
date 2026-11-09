/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp.h"

/* =========================================================================
 * Crash reporting.
 *
 * The old handlers were a bare `while (1)`, which looks exactly like "the
 * program hangs" and tells nobody why.  Now a fault prints the stage marker
 * (see BSP_Stage), the fault status registers and then blinks the onboard
 * LED; if the watchdog is already running, the chip resets itself ~2 s
 * later and the next boot reports "IWDG" in the reset flags, so a hang can
 * be told apart from a power/ wiring problem.
 * ========================================================================= */
static void Fault_Report(const char *name)
{
    BSP_UartSendString("\r\nFAULT ");
    BSP_UartSendString(name);
    BSP_UartSendString(" stage=");
    BSP_UartSendInt((int32_t)BSP_GetStage());
    BSP_UartSendString(" CFSR=");
    BSP_UartSendInt((int32_t)(SCB->CFSR & 0x7FFFFFFFu));
    BSP_UartSendString(" HFSR=");
    BSP_UartSendInt((int32_t)(SCB->HFSR & 0x7FFFFFFFu));
    BSP_UartSendString(" MMFAR=");
    BSP_UartSendInt((int32_t)(SCB->MMFAR & 0x7FFFFFFFu));
    BSP_UartSendString(" BFAR=");
    BSP_UartSendInt((int32_t)(SCB->BFAR & 0x7FFFFFFFu));
    BSP_UartSendString("\r\n");
}

static void Fault_Hold(uint8_t blinks)
{
    volatile uint32_t d;

    for (;;)
    {
        uint8_t i;

        for (i = 0; i < blinks; i++)
        {
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);   /* LED on */
            for (d = 0; d < 1500000u; d++) { }
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
            for (d = 0; d < 1500000u; d++) { }
        }
        for (d = 0; d < 3000000u; d++) { }
    }
}

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  Fault_Report("HARD");
  Fault_Hold(3);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  Fault_Report("MEM");
  Fault_Hold(4);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  Fault_Report("BUS");
  Fault_Hold(5);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  Fault_Report("USAGE");
  Fault_Hold(6);
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
