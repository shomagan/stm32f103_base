#include "step_board.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"
void DELAY_US(uint16_t time_us){
    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_EnableCounter(TIM2);
    static u16 enable_temp = 0;
    static u16 dir_temp = 0;
    while(htim2.Instance->CNT <= time_us){
        if(!HAL_GPIO_ReadPin(CAR_DIRECTION_IN_PORT, CAR_DIRECTION_IN_HAL)){
            if (!dir_temp){
                LL_GPIO_TogglePin(DRV_DIR0_PORT, DRV_DIR0_LL);
                LL_GPIO_TogglePin(DRV_DIR1_PORT, DRV_DIR1_LL);
                dir_temp = 50000;
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
                enable_temp = 50000;
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
    LL_TIM_DisableCounter(TIM2);
}

void step_board_task( const void *parameters){
    (void)parameters;
    while(1){
        LL_GPIO_TogglePin(DRV_STEP0_PORT, DRV_STEP0_LL);
        LL_GPIO_TogglePin(DRV_STEP1_PORT, DRV_STEP1_LL);
        DELAY_US(750);

    }
}

