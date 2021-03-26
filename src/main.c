
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "onewire.h"
#include "control.h"
#include "stm32f1xx_ll_gpio.h"
#include "step.h"
#include "main.h"
#include "tusbd_cdc.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"

#if STEP_BOARD
#include "step_board.h"
#endif

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
static osThreadId own_task_id;
sofi_vars_t sofi;
#if FEEDER
static osThreadId step_task_id;
#endif

#if STEP_BOARD
static osThreadId step_board_task_id;
#else
static osThreadId ds18_task_id,control_task_id;
#endif



/* Private function prototypes -----------------------------------------------*/

static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void RTC_AlarmConfig(void);
void own_task(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
u8 stop_mode = 0;

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void){
    SystemClock_Config();
    HAL_Init();
    MX_GPIO_Init();
#if IWDG_USE
    MX_IWDG_Init();
#endif
#if RTC_USE
    MX_RTC_Init();
#endif
#if ADC_USE
    MX_ADC1_Init();
#endif
#if UART_USE
    MX_USART1_UART_Init();
#endif
#if STEP_BOARD==0
    MX_TIM3_Init();
#endif
    MX_TIM2_Init();
    osThreadDef(own_task, own_task, osPriorityNormal, 0, 364);
    own_task_id = osThreadCreate(osThread(own_task), NULL);

#if STEP_BOARD
    osThreadDef(step_board_task, step_board_task, osPriorityNormal, 0, 364);
    step_board_task_id = osThreadCreate(osThread(step_board_task), NULL);

#else
    #if FEEDER
        osThreadDef(step_task, step_task, osPriorityNormal, 0, 364);
        step_task_id = osThreadCreate(osThread(step_task), NULL);
    #endif
    osThreadDef(ds18_task, ds18_task, osPriorityNormal, 0, 364);
    ds18_task_id = osThreadCreate(osThread(ds18_task), NULL);
    osThreadDef(control_task, control_task, osPriorityNormal, 0, 364);
    control_task_id = osThreadCreate(osThread(control_task), NULL);
#endif
    /* Start scheduler */
    HAL_TIM_Base_Stop(&htim1);
    osKernelStart();
    while (1)  { }
}
void pre_sleep_proccess(){
    HAL_TIM_Base_Stop(&htim1);
}
void post_sleep_proccess(){
    HAL_TIM_Base_Start(&htim1);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    /**Initializes the CPU, AHB and APB busses clocks*/

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
#if RTC_USE
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
#else
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
#endif

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
    /**Initializes the CPU, AHB and APB busses clocks   */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    PeriphClkInit.PeriphClockSelection =0;
    PeriphClkInit.PeriphClockSelection |=
#if USB_USE
            RCC_PERIPHCLK_USB|
    #if ADC_USE
            RCC_PERIPHCLK_ADC|
    #endif
#else
    #if ADC_USE
            RCC_PERIPHCLK_ADC|
    #endif
#endif
    0;
#if ADC_USE
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
#endif
#if USB_USE
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
    /**Configure the Systick interrupt time*/
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    /**Configure the Systick*/
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 8, 0);
}
__STATIC_INLINE void SYSCLKConfig_FromSTOP(void){
  /* Customize process using LL interface to improve the performance
     (wake-up time from STOP quicker in LL than HAL)*/
  /* HSE configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1) {}
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1) {}
  /* Main PLL activation */
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1) {}
  /* SYSCLK activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){}
  HAL_RTCEx_SetSecond_IT(&hrtc);
  HAL_NVIC_SetPriority(RTC_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);

}

/* ADC1 init function */
static void MX_ADC1_Init(void){
    ADC_ChannelConfTypeDef sConfig;
    ADC_InjectionConfTypeDef sConfigInjected;
    /**Common config*/
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = ENABLE;
    hadc1.Init.NbrOfDiscConversion = 1;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Regular Channel*/
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Injected Channel*/
    /*common*/
    sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
    sConfigInjected.InjectedNbrOfConversion = 4;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
    sConfigInjected.InjectedOffset = 0;
    /*specific*/
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Injected Channel*/
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Injected Channel*/
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Injected Channel*/
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* IWDG init function */
static void MX_IWDG_Init(void){
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* RTC init function */
static void MX_RTC_Init(void){
    RTC_TimeTypeDef sTime;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
    HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
    u32 data;
    const  u32 data_c = 0x1234;
    data = BKP->DR1;
    if(data!=data_c){
        BKP->DR1 = data_c;
        sTime.Hours = 21;
        sTime.Minutes = 15;
        sTime.Seconds = 0;
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
    }
    data = BKP->DR1;
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
    RTC_AlarmConfig();
}
static void RTC_AlarmConfig(void){
  RTC_AlarmTypeDef salarmstructure = {{0}, 0};
  RTC_TimeTypeDef sTime;
  HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
  if (sTime.Seconds>=58){
      sTime.Seconds = 0;
      if(sTime.Minutes == 59){
          sTime.Minutes = 0;
          if(sTime.Hours == 23){
             sTime.Hours = 0;
          }else{
             sTime.Hours += 1;
          }
      }else{
          sTime.Minutes += 1;
      }
  }else{
      sTime.Seconds+=2;
  }

  /*##-3- Configure the RTC Alarm peripheral #################################*/
  /* Set Alarm to 00:00:10
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  salarmstructure.Alarm = RTC_ALARM_A;
  salarmstructure.AlarmTime.Hours = sTime.Hours;
  salarmstructure.AlarmTime.Minutes = sTime.Minutes;
  salarmstructure.AlarmTime.Seconds = sTime.Seconds;
  if(HAL_RTC_SetAlarm_IT(&hrtc,&salarmstructure,RTC_FORMAT_BIN) != HAL_OK){
    /* Initialization Error */
    Error_Handler();
  }
}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
    (void)hrtc;
    RTC_AlarmConfig();
}
/* TIM3 init function */
static void MX_TIM3_Init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef pwm_handle;
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = MAX_PWM_VALUE;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    pwm_handle.OCMode = TIM_OCMODE_PWM1;
    pwm_handle.Pulse = 16000;
    pwm_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_handle.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_handle, TIM_CHANNEL_1) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    HAL_TIM_MspPostInit(&htim3);
}


/* USART1 init function */
static void MX_USART1_UART_Init(void){
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/** Configure pins as 
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void){
    /* GPIO Ports Clock Enable */
#if USE_GPIO_PORT_C
    __HAL_RCC_GPIOC_CLK_ENABLE();
#endif
#if USE_GPIO_PORT_D
    __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#if USE_GPIO_PORT_A
    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif
#if USE_GPIO_PORT_B
    __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
#if STEP_BOARD
    LL_GPIO_SetPinMode(DRV_STEP0_PORT, DRV_STEP0_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_STEP0_PORT, DRV_STEP0_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(DRV_STEP0_PORT, DRV_STEP0_LL);
    LL_GPIO_SetPinMode(DRV_DIR0_PORT, DRV_DIR0_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_DIR0_PORT, DRV_DIR0_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(DRV_DIR0_PORT, DRV_DIR0_LL);
    LL_GPIO_SetPinMode(DRV_STEP1_PORT, DRV_STEP1_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_STEP1_PORT, DRV_STEP1_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(DRV_STEP1_PORT, DRV_STEP1_LL);
    LL_GPIO_SetPinMode(DRV_DIR1_PORT, DRV_DIR1_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_DIR1_PORT, DRV_DIR1_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_ResetOutputPin(DRV_DIR1_PORT, DRV_DIR1_LL);

/*portb*/
    LL_GPIO_SetPinMode(DRV_SLIP_PORT, DRV_SLIP_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_SLIP_PORT, DRV_SLIP_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(DRV_SLIP_PORT, DRV_SLIP_LL);

    LL_GPIO_SetPinMode(DRV_RST_PORT, DRV_RST_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_RST_PORT, DRV_RST_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(DRV_RST_PORT, DRV_RST_LL);

    LL_GPIO_SetPinMode(DRV_FLT_PORT, DRV_FLT_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_FLT_PORT, DRV_FLT_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(DRV_FLT_PORT, DRV_FLT_LL);

    LL_GPIO_SetPinMode(DRV_EN_PORT, DRV_EN_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DRV_EN_PORT, DRV_EN_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(DRV_EN_PORT, DRV_EN_LL);
    LL_GPIO_SetPinMode(LED_PORT, LED_PIN, LL_GPIO_MODE_OUTPUT);
#if CAR_KIDS
    GPIO_InitTypeDef	gpinit;
    gpinit.Mode = GPIO_MODE_INPUT;
    gpinit.Pull = GPIO_PULLUP;
    gpinit.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpinit.Pin = CAR_DIRECTION_IN_HAL;
    HAL_GPIO_Init(CAR_DIRECTION_IN_PORT,&gpinit);
    gpinit.Pin = CAR_SPEED_IN_HAL;
    HAL_GPIO_Init(CAR_SPEED_IN_PORT,&gpinit);
    gpinit.Pin = CAR_ENGINE_IN_HAL;
    HAL_GPIO_Init(CAR_ENGINE_IN_PORT,&gpinit);

    LL_GPIO_SetPinMode(CAR_DIRECTION_OUT_PORT, CAR_DIRECTION_OUT_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(CAR_DIRECTION_OUT_PORT, CAR_DIRECTION_OUT_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_ResetOutputPin(CAR_DIRECTION_OUT_PORT, CAR_DIRECTION_OUT_LL);
    LL_GPIO_SetPinMode(CAR_SPEED_OUT_PORT, CAR_SPEED_OUT_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(CAR_SPEED_OUT_PORT, CAR_SPEED_OUT_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_ResetOutputPin(CAR_SPEED_OUT_PORT, CAR_SPEED_OUT_LL);
    LL_GPIO_SetPinMode(CAR_ENGINE_OUT_PORT, CAR_ENGINE_OUT_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(CAR_ENGINE_OUT_PORT, CAR_ENGINE_OUT_LL, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_ResetOutputPin(CAR_ENGINE_OUT_PORT, CAR_ENGINE_OUT_LL);

#endif

#else
    LL_GPIO_SetPinMode(AIR_PORT, AIR_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(FLOW_PORT, FLOW_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LIGTH1_PORT, LIGTH1_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LIGTH2_PORT, LIGTH2_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_PORT, LED_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(STEP_PORT, STEP_OUT1_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(STEP_PORT, STEP_OUT1_1,LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(STEP_PORT, STEP_OUT1_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(STEP_PORT, STEP_OUT1_2,LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(STEP_PORT, STEP_OUT2_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(STEP_PORT, STEP_OUT2_1,LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(STEP_PORT, STEP_OUT2_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(STEP_PORT, STEP_OUT2_2,LL_GPIO_OUTPUT_PUSHPULL);
#endif
    /*DS18b20 sensor*/
    LL_GPIO_SetPinMode(DS18B20_GPIO, DS18B20_PIN_LL, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(DS18B20_GPIO, DS18B20_PIN_LL, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetOutputPin(DS18B20_GPIO, DS18B20_PIN_LL);
}
#if USB_USE
// The CDC recv buffer size should equal to the out endpoint size
// or we will need a timeout to flush the recv buffer
#define CDC_RX_EP_SIZE    32
static __ALIGN_BEGIN uint8_t cdc_buf[CDC_RX_EP_SIZE] __ALIGN_END;

int cdc_recv_data(tusb_cdc_device_t* cdc, const void* data, uint16_t len);
int cdc_send_done(tusb_cdc_device_t* cdc);
void cdc_line_coding_change(tusb_cdc_device_t* cdc);

static tusb_cdc_device_t cdc_dev = {
  .backend = &cdc_device_backend,
  .ep_in = 1,
  .ep_out = 1,
  .ep_int = 8,
  .on_recv_data = cdc_recv_data,
  .on_send_done = cdc_send_done,
  .on_line_coding_change = cdc_line_coding_change,
  .rx_buf = cdc_buf,
  .rx_size = sizeof(cdc_buf),
};
static int cdc_len = 0;
void cdc_line_coding_change(tusb_cdc_device_t* cdc){
  // TODO, handle the line coding change
  //cdc->line_coding.bitrate;
  //cdc->line_coding.databits;
  //cdc->line_coding.stopbits;
  //cdc->line_coding.parity;
}

int cdc_recv_data(tusb_cdc_device_t* cdc, const void* data, uint16_t len){
    cdc_len = (int)len;
    return 1; // return 1 means the recv buffer is busy
}

int cdc_send_done(tusb_cdc_device_t* cdc){
    tusb_set_rx_valid(cdc->dev, cdc->ep_out);
    return 0;
}
// make sure the interface order is same in "composite_desc.lua"
static tusb_device_interface_t* device_interfaces[] = {
    (tusb_device_interface_t*)&cdc_dev, 0,   // CDC need two interfaces
};

static void init_ep(tusb_device_t* dev){
    CDC_ACM_TUSB_INIT(dev);
}
void tusb_delay_ms(uint32_t ms){
    osDelay(ms);
}


static tusb_device_config_t device_config = {
    .if_count = sizeof(device_interfaces)/sizeof(device_interfaces[0]),
    .interfaces = &device_interfaces[0],
    .ep_init = init_ep,
};
#endif
/* StartDefaultTask function */
static u32 test1 = 0;
void own_task(void const * argument){
    (void)argument;
#if IWDG_USE
    HAL_IWDG_Refresh(&hiwdg);
#endif
#if USB_USE
    tusb_device_t* dev = tusb_get_device(TEST_APP_USB_CORE);
    tusb_set_device_config(dev, &device_config);
    tusb_open_device(dev);
#endif

    while(1){
#if IWDG_USE
        HAL_IWDG_Refresh(&hiwdg);
#endif
/*        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
        HAL_SuspendTick();
        EXTI->PR = 0xFFFFFFFF;
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
        SYSCLKConfig_FromSTOP();
        HAL_ResumeTick();
        test1++;*/
        LL_GPIO_TogglePin(LED_PORT, LED_PIN);
#if USB_USE
        if(cdc_len){
            if (cdc_buf[0]==0x01){
                LL_GPIO_TogglePin(DRV_DIR0_PORT, DRV_DIR0_LL);
                LL_GPIO_TogglePin(DRV_DIR1_PORT, DRV_DIR1_LL);
            }
            tusb_cdc_device_send(&cdc_dev, cdc_buf, (u16)cdc_len);
            cdc_len = 0;
        }
#endif
        osDelay(1000);
    }
}



/* TIM2 init function */
static void MX_TIM2_Init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig={0};
    TIM_MasterConfigTypeDef sMasterConfig={0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (((HAL_RCC_GetPCLK2Freq())/ 1000000) - 1);//63
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xffff;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sClockSourceConfig.ClockFilter = 0;
    sClockSourceConfig.ClockPrescaler = 0;
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    return;
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line){
    while(1){
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){ 
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

