/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__
#include "type_def.h"
#define SOFI_MONITOR 1
#define MONITOR_MAX_TASKS 10
/*port a*/
#define TRIGER_CONTROL_WITHOUT_PWM 1
#define PID_OUT_PORT_0 GPIOA
#define PID_OUT_PIN_0 LL_GPIO_PIN_6
#define PID_OUT_PIN_HAL_0 GPIO_PIN_6

#define AIR_PORT GPIOA
#define AIR_PIN  LL_GPIO_PIN_7
#define FLOW_PORT GPIOA
#define FLOW_PIN  LL_GPIO_PIN_5
#define LIGTH1_PORT GPIOA
#define LIGTH1_PIN  LL_GPIO_PIN_4
#define LIGTH2_PORT GPIOA
#define LIGTH2_PIN  LL_GPIO_PIN_3
#define ADC0_PIN  GPIO_PIN_0
#define ADC1_PIN  GPIO_PIN_1
#define ADC_PORT GPIOA

/**
 * USART1 GPIO Configuration
 * PA9     ------> USART1_TX
 * PA10     ------> USART1_RX
 */

/*port b */
#define	DS18B20_GPIO  GPIOB
#define	DS18B20_PIN   GPIO_PIN_15
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL  GPIO_PIN_8
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA  GPIO_PIN_9
#define STEP_OUT1_1 LL_GPIO_PIN_13
#define STEP_OUT1_2 LL_GPIO_PIN_14
#define STEP_OUT2_1 LL_GPIO_PIN_11
#define STEP_OUT2_2 LL_GPIO_PIN_12
#define STEP_PORT GPIOB
#define PID_OUT_PORT_3 GPIOB
#define PID_OUT_PIN_3 LL_GPIO_PIN_6
#define PID_OUT_PIN_HAL_3 GPIO_PIN_6
/*port c */
#define LED_PORT GPIOC
#define LED_PIN  LL_GPIO_PIN_13

extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
#define GLOBAL_UNION_SIZE 64
/*
 * @brief name variables uses for name in description file and then in get value by name
 * and therefore use max size len name is 16 charackter
 * @coment style : "" - description
 *                  &ro  - read only
 *                  &def -> have const varibale with struct like def_name
 *                  &save- will have saved in bkram
 *
 * */
typedef union{
    struct MCU_PACK{
        u32 cur_free_heap;              //!< in bytes,&ro
        u32 min_free_heap;              //!< in bytes,&ro
        u8 debug_info[8];               //!<"reserved use for debug"
        //add new regs after ->
        u8 user_task_state;             //!<"user task current state",&ro
        u16 user_task_config;           //!<"user task config",
        //add new regs before <-
        u32 monitor_period;             //!< "sofi_monitor period in ms",&ro
        float total_tasks_time;         //!< "sum of running times of tasks in %",&ro
    }vars;
    u8 bytes[GLOBAL_UNION_SIZE]; //for full bksram copy
}sofi_vars_t;
typedef struct {
    u8 task_number;
    u32 last_time;
    float percentage;
}task_time_t;
typedef enum {
    ERROR_OK = 0,
    ERROR_INIT = -1
}error_t;

extern sofi_vars_t sofi;


/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
