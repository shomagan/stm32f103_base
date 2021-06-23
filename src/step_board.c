#include "step_board.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"
static u16 time_to_going = 0;
#define SIMPLY_AERO 0
#if SIMPLY_AERO
static u32 tick_timer;
#endif
void DELAY_US(uint16_t time_us){
    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_EnableCounter(TIM2);
    static u16 enable_temp = 0;
    static u16 dir_temp = 0;
#if SIMPLY_AERO
    while(htim2.Instance->CNT <= time_us){
        osThreadYield();
    }
#else
    while(htim2.Instance->CNT <= time_us){
        if(!HAL_GPIO_ReadPin(CAR_DIRECTION_IN_PORT, CAR_DIRECTION_IN_HAL)){
            if (!dir_temp){
                LL_GPIO_TogglePin(DRV_DIR0_PORT, DRV_DIR0_LL);
                LL_GPIO_TogglePin(DRV_DIR1_PORT, DRV_DIR1_LL);
                time_to_going = 0;
                dir_temp = 65000;
            }else{
                dir_temp--;
            }

        }else{
            if(dir_temp){
                dir_temp--;
            }
        }
        if(!HAL_GPIO_ReadPin(CAR_ENGINE_IN_PORT, CAR_ENGINE_IN_HAL)){
            if (!enable_temp){
                LL_GPIO_TogglePin(DRV_EN_PORT, DRV_EN_LL);
                time_to_going=0;
                enable_temp = 65000;
            }else{
                enable_temp--;
            }
        }else{
            if(enable_temp){
                enable_temp--;
            }
        }
        osThreadYield();
    }
#endif
    LL_TIM_DisableCounter(TIM2);
}

void car_step_board_task( const void *parameters){
    (void)parameters;
#if SIMPLY_AERO
    LL_GPIO_ResetOutputPin(DRV_DIR0_PORT, DRV_DIR0_LL);
    LL_GPIO_SetOutputPin(DRV_DIR1_PORT, DRV_DIR1_LL);
#endif
    while(1){
        LL_GPIO_TogglePin(DRV_STEP0_PORT, DRV_STEP0_LL);
        LL_GPIO_TogglePin(DRV_STEP1_PORT, DRV_STEP1_LL);
#if SIMPLY_AERO
        tick_timer = osKernelSysTick();
        if ((tick_timer%3600000) < 300000){
            LL_GPIO_ResetOutputPin(DRV_EN_PORT, DRV_EN_LL);
        }else{
            LL_GPIO_SetOutputPin(DRV_EN_PORT, DRV_EN_LL);
        }
        DELAY_US(1500);
#else
        DELAY_US(1600-time_to_going);
#endif
        if(time_to_going<400){
            time_to_going+=1;
        }
    }
}

