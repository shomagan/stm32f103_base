
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
#include "ds18.h"
#include "control.h"
#include "stm32f1xx_ll_gpio.h"
#include "step.h"
#include "main.h"
#include "tusbd_cdc.h"
#define FEEDER 1
#define ADC_USE 0

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
static osThreadId own_task_id;

#if FEEDER
static osThreadId step_task_id;
#endif
static osThreadId ds18_task_id,control_task_id;




/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void own_task(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_IWDG_Init();
    MX_RTC_Init();
#if ADC_USE
    MX_ADC1_Init();
#endif
    MX_USART1_UART_Init();
    MX_TIM3_Init();
    MX_TIM2_Init();
    osThreadDef(own_task, own_task, osPriorityNormal, 0, 364);
    own_task_id = osThreadCreate(osThread(own_task), NULL);
#if FEEDER
    osThreadDef(step_task, step_task, osPriorityNormal, 0, 364);
    step_task_id = osThreadCreate(osThread(step_task), NULL);
#endif
    osThreadDef(ds18_task, ds18_task, osPriorityNormal, 0, 364);
    ds18_task_id = osThreadCreate(osThread(ds18_task), NULL);
    osThreadDef(control_task, control_task, osPriorityNormal, 0, 364);
    control_task_id = osThreadCreate(osThread(control_task), NULL);

    /* Start scheduler */
    osKernelStart();
    while (1)  { }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    /**Initializes the CPU, AHB and APB busses clocks*/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
            |RCC_PERIPHCLK_USB;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
    /**Configure the Systick interrupt time*/
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    /**Configure the Systick*/
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
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
    sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedNbrOfConversion = 4;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
    sConfigInjected.InjectedOffset = 0;
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
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
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
        sTime.Hours = 0x01;
        sTime.Minutes = 0x01;
        sTime.Seconds = 0x01;
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
    }
    data = BKP->DR1;
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
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
}
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

/* StartDefaultTask function */
void own_task(void const * argument){
    (void)argument;
    HAL_IWDG_Refresh(&hiwdg);
    tusb_device_t* dev = tusb_get_device(TEST_APP_USB_CORE);
    tusb_set_device_config(dev, &device_config);
    tusb_open_device(dev);

    while(1){
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(10);
        if(cdc_len){
            tusb_cdc_device_send(&cdc_dev, cdc_buf, (u16)cdc_len);
            cdc_len = 0;
        }
    }
}
/* TIM2 init function */
static void MX_TIM2_Init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig={0};
    TIM_MasterConfigTypeDef sMasterConfig;
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65500;
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

