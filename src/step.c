#include "step.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f1xx_ll_gpio.h"

static void step_out_1_disable(void);
static void step_out_2_disable(void);
static void step_out_1_forward(void);
static void step_out_2_forward(void);
static void step_out_1_reverse(void);
static void step_out_2_reverse(void);
static void forward(int time);
static void reverse(int time);
#define STEP_DELAY 2

void step_task( const void *parameters){
    (void)parameters;
    uint32_t kernel_tick = osKernelSysTick();
    RTC_TimeTypeDef time;
    step_out_1_disable();
    step_out_2_disable();
    while(1){

        HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
        u32 current_sec = time.Hours*3600 + time.Minutes*60 + time.Seconds;
        if((68461< current_sec) && (current_sec< 68481)){
            forward(20);
            kernel_tick = osKernelSysTick();
            osDelayUntil(&kernel_tick, 20000);
        }
        if((10861< current_sec) && (current_sec< 10881)){
            forward(20);
            kernel_tick = osKernelSysTick();
            osDelayUntil(&kernel_tick, 20000);
        }

        osDelayUntil(&kernel_tick, 2000);
    }
}
/*@time in ms(% 4)
 *@brief refresh watchdog timer
 * */
void forward(int time){
    u32 kernel_tick;
    kernel_tick = osKernelSysTick();
    step_out_1_disable();
    step_out_2_disable();
    step_out_2_reverse();
    for (int i=0;i<time;i++){
        step_out_1_forward();
        osDelayUntil(&kernel_tick, STEP_DELAY);
        step_out_2_forward();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&kernel_tick, STEP_DELAY);
        step_out_1_reverse();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&kernel_tick, STEP_DELAY);
        step_out_2_reverse();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&kernel_tick, STEP_DELAY);
        HAL_IWDG_Refresh(&hiwdg);
    }
    step_out_1_disable();
    step_out_2_disable();

}
/*@time in ms(% 4)
 *@brief refresh watchdog timer
 * */
void reverse(int time){
    u32 kernel_tick;
    kernel_tick = osKernelSysTick();
    step_out_1_disable();
    step_out_2_disable();
    step_out_1_reverse();
    for (int i=0;i<time;i++){
        step_out_2_forward();
        osDelayUntil(&kernel_tick, STEP_DELAY);
        HAL_IWDG_Refresh(&hiwdg);
        step_out_1_forward();
        osDelayUntil(&kernel_tick, STEP_DELAY);
        HAL_IWDG_Refresh(&hiwdg);
        step_out_2_reverse();
        osDelayUntil(&kernel_tick, STEP_DELAY);
        HAL_IWDG_Refresh(&hiwdg);
        step_out_1_reverse();
        osDelayUntil(&kernel_tick, STEP_DELAY);
        HAL_IWDG_Refresh(&hiwdg);
    }
    step_out_1_disable();
    step_out_2_disable();

}

void step_out_1_disable(){
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT1_1);
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT1_2);
}
void step_out_2_disable(){
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT2_1);
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT2_2);
}
void step_out_1_forward(){
    LL_GPIO_SetOutputPin(STEP_PORT,STEP_OUT1_1);
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT1_2);
}
void step_out_2_forward(){
    LL_GPIO_SetOutputPin(STEP_PORT,STEP_OUT2_1);
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT2_2);
}
void step_out_1_reverse(){
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT1_1);
    LL_GPIO_SetOutputPin(STEP_PORT,STEP_OUT1_2);
}
void step_out_2_reverse(){
    LL_GPIO_ResetOutputPin(STEP_PORT,STEP_OUT2_1);
    LL_GPIO_SetOutputPin(STEP_PORT,STEP_OUT2_2);
}
