/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"
#include "main.h"

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void){
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void){
  while (1){
  }
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void){
  while (1){
  }
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void){
  while (1){
  }
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void){
  while (1)  {
  }
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void){
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void){
  HAL_IncTick();
  osSystickHandler();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RTC global interrupt.
*/
static u32 test = 0;
void RTC_IRQHandler(void){
  HAL_RTCEx_RTCIRQHandler(&hrtc);
  test++;
}
/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */
void RTC_Alarm_IRQHandler(void){
  HAL_RTC_AlarmIRQHandler(&hrtc);

}

/**
* @brief This function handles USB wake-up interrupt through EXTI line 18.
*/
void USBWakeUp_IRQHandler(void){
/*  if ((&hpcd_USB_FS)->Init.low_power_enable) {

    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    SystemClock_Config();
  }
  __HAL_USB_WAKEUP_EXTI_CLEAR_FLAG();*/
}

/**
* @brief This function handles TIM1 update interrupt. using hal lib
*/
void TIM1_UP_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim1);
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim3);
}

/**
* @brief This function handles TIM2 global interrupt.
*/

void TIM2_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim2);
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void){
  HAL_UART_IRQHandler(&huart1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
