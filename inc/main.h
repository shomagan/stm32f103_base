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

#define STEP_BOARD 1
#define CAR_KIDS 1
#define FEEDER 0
#define ADC_USE 0
#define CONTROL_TASK 1
#define DS18_TASK 1

/*porta*/
#define DRV_STEP0_LL LL_GPIO_PIN_7
#define DRV_STEP0_HAL GPIO_PIN_7
#define DRV_STEP0_PORT GPIOA
#define DRV_DIR0_LL LL_GPIO_PIN_6
#define DRV_DIR0_HAL GPIO_PIN_6
#define DRV_DIR0_PORT GPIOA
#define DRV_STEP1_LL LL_GPIO_PIN_5
#define DRV_STEP1_HAL GPIO_PIN_5
#define DRV_STEP1_PORT GPIOA
#define DRV_DIR1_LL LL_GPIO_PIN_4
#define DRV_DIR1_HAL GPIO_PIN_4
#define DRV_DIR1_PORT GPIOA

#define DRV_STEP2_LL LL_GPIO_PIN_3
#define DRV_STEP2_HAL GPIO_PIN_3
#define DRV_STEP2_PORT GPIOA
#define CAR_DIRECTION_IN_LL DRV_STEP2_LL
#define CAR_DIRECTION_IN_HAL DRV_STEP2_HAL
#define CAR_DIRECTION_IN_PORT DRV_STEP2_PORT


#define DRV_DIR2_LL LL_GPIO_PIN_2
#define DRV_DIR2_HAL GPIO_PIN_2
#define DRV_DIR2_PORT GPIOA
#define CAR_DIRECTION_OUT_LL DRV_DIR2_LL
#define CAR_DIRECTION_OUT_HAL DRV_DIR2_HAL
#define CAR_DIRECTION_OUT_PORT DRV_DIR2_PORT


#define DRV_STEP3_LL LL_GPIO_PIN_15
#define DRV_STEP3_HAL GPIO_PIN_15
#define DRV_STEP3_PORT GPIOB
#define CAR_SPEED_IN_LL DRV_STEP3_LL
#define CAR_SPEED_IN_HAL DRV_STEP3_HAL
#define CAR_SPEED_IN_PORT DRV_STEP3_PORT

#define DRV_DIR3_LL LL_GPIO_PIN_8
#define DRV_DIR3_HAL GPIO_PIN_8
#define DRV_DIR3_PORT GPIOA
#define CAR_SPEED_OUT_LL DRV_DIR3_LL
#define CAR_SPEED_OUT_HAL DRV_DIR3_HAL
#define CAR_SPEED_OUT_PORT DRV_DIR3_PORT


#define CAN_RX_LL LL_GPIO_PIN_8
#define CAN_RX_HAL GPIO_PIN_8
#define CAN_RX_PORT GPIOB
#define CAR_ENGINE_OUT_LL CAN_RX_LL
#define CAR_ENGINE_OUT_HAL CAN_RX_HAL
#define CAR_ENGINE_OUT_PORT CAN_RX_PORT


#define CAN_TX_LL LL_GPIO_PIN_9
#define CAN_TX_HAL GPIO_PIN_9
#define CAN_TX_PORT GPIOB
#define CAR_ENGINE_IN_LL CAN_TX_LL
#define CAR_ENGINE_IN_HAL CAN_TX_HAL
#define CAR_ENGINE_IN_PORT CAN_TX_PORT



/*portb*/
#define DRV_SLIP_LL LL_GPIO_PIN_12
#define DRV_SLIP_HAL GPIO_PIN_12
#define DRV_SLIP_PORT GPIOB
#define DRV_RST_LL LL_GPIO_PIN_13
#define DRV_RST_HAL GPIO_PIN_13
#define DRV_RST_PORT GPIOB
#define DRV_FLT_LL LL_GPIO_PIN_14
#define DRV_FLT_HAL GPIO_PIN_14
#define DRV_FLT_PORT GPIOB
#define DRV_EN_LL LL_GPIO_PIN_0
#define DRV_EN_HAL GPIO_PIN_0
#define DRV_EN_PORT GPIOB


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
#define	DS18B20_PIN_LL   LL_GPIO_PIN_15
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
extern u8 stop_mode;
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
void SystemClock_Config(void);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
