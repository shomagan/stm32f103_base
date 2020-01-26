/**
 * @file control.c
 * @author Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 * @defgroup src
 * @ingroup src
 * @version 0.1
 * @brief  TODO!!! write brief in
 */
/*
 * Copyright (c) 2018 Snema Service
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the sofi PLC.
 *
 * Author: Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 */
#ifndef CONTROL_C
#define CONTROL_C 1
#include "control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
//#include "usbd_cdc_if.h"
#include "ds18.h"
#include "ssd1306.h"
#include "time_table.h"
#include "stm32f1xx_ll_gpio.h"
#include "main.h"
/* fb pid */
typedef union DataTypes_union{
    u8 bit:1;
    u8 uint8;
    u16 uint16;
    u32 uint32;
    float float32;
    u8 array[4];
} data_types;

typedef struct __attribute__((packed)){
    u8 type;
    data_types data;
} register_type;

typedef struct {
    register_type enable;				// bit 0 - Ручное, 1 - Автоматическое
    register_type reverse_control;			// bit 1- реверсивное управление
    register_type rezet;			// bit 1- сброс накопленных параметров
    register_type require_value; 		// float Уставка регулирования
    register_type current_value;			// float Регулируемый параметр
    register_type kp;		 		// float Коэффициент пропорциональности
    register_type ki;		  		// float Коэффициент времени интегрирования
    register_type kd;				// float Коэффициент времени интегрирования
    register_type position;	    	// float - необходимое положение регулятора в процентах
    register_type gist_tube;	 		// float Зона нечувствительности в единицах измеряемого параметра
} fb00099_IN_type;

typedef struct {
    register_type error_integral;		// float - накопленная ошибка интегратора
    register_type prev_error_integral;			// float - предыдущее значение ошибки регулирования
    register_type prev_control_integral;			// float - накопленное воздействия на регулирующий орган
    register_type enable_old;			// bit - для отслеживания первого такта включения
    register_type number_tick;			// uint32 - количество тактов после включения,для интервала работы
} fb00099_VAR_type;

typedef struct {
    register_type error;	     	// bit Индикация ошибки входных параметров
    register_type output;	    	// float - необходимое положение регулятора в процентах
    register_type test;				// float
} fb00099_OUT_type;

void fb00099_exec(fb00099_IN_type * FBInputs,fb00099_VAR_type * FBVars,\
                  fb00099_OUT_type * FBOutputs);

#define DEFAULT_OUT 0.0f
#define REQUIRE_VALUE 27.0f

static void set_pwm_value(float value);
static u8 check_state_machine(void);
static void air_do_control(u8 enable);
static void flow_do_control(u8 enable);
static void ligth_do_control(u8 enable);
void ligth2_do_control(u8 enable);
typedef struct __attribute__((__packed__)){
    u16 number;
    u32 stop_time;  //in seconde
}state_machine;
static const time_table_t time_table_flow[] = {
    {600,1200},
    {810,1200},
    {1020,1200},
};

static const time_table_t time_table_ligth[] = {
    {600,1500},
    {630,1500},
    {660,1500},
    {690,1500},
    {720,1500},
    {750,1500},
    {780,1500},
    {810,1500},
    {840,1500},
    {870,1500},
    {900,1500},
    {930,1500},
    {960,1500},
    {990,1500},
    {1020,1500},
    {1050,1500},
    {1080,1500},
    {1110,1500},
    {1140,1500},
    {1170,1500},
};
static const time_table_t time_table_ligth2[] = {
    {600,1500},
    {630,1500},
    {660,1500},
    {690,1500},
    {720,1500},
    {750,1500},
    {780,1500},
    {810,1500},
    {840,1500},
    {870,1500},
    {900,1500},
    {930,1500},
    {960,1500},
    {990,1500},
    {1020,1500},
    {1050,1500},
    {1080,1500},
    {1110,1500},
    {1140,1500},
    {1170,1500},
};

static const time_table_t time_table_air[] = {
    {602,3600},
    {840,3600},
    {1202,3600}
};
static state_machine flow,ligth,ligth2,air;
void control_task( const void *parameters){
    u32 tick=0;
    fb00099_IN_type in;
    fb00099_VAR_type var;
    fb00099_OUT_type out;
    var.prev_error_integral.data.float32 = 0.0;
    var.error_integral.data.float32  = 0.0;
    var.number_tick.data.uint32=0.0;
    in.enable.data.bit = 1;
    in.reverse_control.data.bit = 0;
    in.rezet.data.bit = 0;
    in.require_value.data.float32 = REQUIRE_VALUE;
    in.current_value.data.float32 = REQUIRE_VALUE;
    in.kp.data.float32 = 75.0f;
    in.ki.data.float32 = 9.0f;
    in.kd.data.float32 = -100.0f;
    in.position.data.float32 = DEFAULT_OUT;
    in.gist_tube.data.float32 = 0.5f;
    taskENTER_CRITICAL();
    HAL_IWDG_Refresh(&hiwdg);
    SSD1306_Init();
    HAL_IWDG_Refresh(&hiwdg);
    //SSD1306_DrawCircle(10, 33, 7, SSD1306_COLOR_WHITE,0.5);
    SSD1306_UpdateScreen();
    taskEXIT_CRITICAL();
    flow.stop_time = 0;
    ligth.stop_time = 0;
    ligth2.stop_time = 0;
    air.stop_time = 0;
    while(1){
        //ds18_time = osKernelSysTick();
        u8 sensor_data_valid;
        tick++;
        sensor_data_valid = 0;
        for (uint8_t i = 0; i < temp_sensor_count; i++){
            if(ds18b20[i].data_validate) {
                in.current_value.data.float32  = ds18b20[i].Temperature;
            }
        }
        fb00099_exec(&in,&var,&out);
        if(data_valid) {
            static float value;
            char buff[32];
            value = out.output.data.float32 >= 0.0f?out.output.data.float32:0.0f;
            value = value <= 75.0f?value:75.0f;
            set_pwm_value(value);
            in.position.data.float32 = out.output.data.float32;
            sprintf(buff,"t - %3.1f, pwm - %3.1f",in.current_value.data.float32,value);
            SSD1306_GotoXY(0, 44);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_UpdateScreen();
        }else{
            char buff[32];
            sprintf(buff,"temperature off");

            SSD1306_GotoXY(0, 44); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
            SSD1306_UpdateScreen();
            set_pwm_value(DEFAULT_OUT);
            in.position.data.float32 = DEFAULT_OUT;	    	// float - необходимое положение регулятора в процентах
        }
        if(SSD1306.error_num){
            SSD1306.Initialized = 0;
        }
        if((tick % 5) == 0){
            if(!SSD1306.Initialized ){
                SSD1306_Init();
            }
        }
        check_state_machine();
        osDelay(1000);
        HAL_IWDG_Refresh(&hiwdg);
        //osDelayUntil(&ds18_time,_DS18B20_UPDATE_INTERVAL_MS);
    }
}
u8 check_state_machine(){
    u16 item_number;
    RTC_TimeTypeDef time;
    u32 temp_sec;
    u16 i;
    LL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
    char buff[32];
    sprintf(buff,"time %2u:%2u:%2u",time.Hours,time.Minutes,time.Seconds);
    SSD1306_GotoXY(0, 0); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
    SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
    SSD1306_UpdateScreen();

    if((time.Hours==0) && (time.Minutes==0)){
        flow.stop_time = 0;
        ligth.stop_time = 0;
        ligth2.stop_time = 0;
        air.stop_time = 0;
        air_do_control(0);
        flow_do_control(0);
        ligth_do_control(0);
        ligth2_do_control(0);
    }else{
        u32 current_sec = time.Hours*3600 + time.Minutes*60 + time.Seconds;
        if(air.stop_time < current_sec){
            item_number = sizeof(time_table_air)/sizeof (time_table_t);
            for (i=0;i<item_number;i++){
                temp_sec = time_table_air[i].start_time*60;
                if((current_sec >= temp_sec) &&
                    (current_sec <=(temp_sec + time_table_air[i].length))){
                    air.stop_time = temp_sec + time_table_air[i].length;
                    air.number = i;
                    air_do_control(1);
                    break;
                }
                if(i>=item_number){
                    air.number = i;
                    air.stop_time =0;
                }
            }
        }
        if(flow.stop_time < current_sec){
            item_number = sizeof(time_table_flow)/sizeof (time_table_t);
            for (u16 i=0;i<item_number;i++){
                temp_sec = time_table_flow[i].start_time*60;
                if((current_sec >= temp_sec) &&
                    (current_sec <=(temp_sec + time_table_flow[i].length))){
                    flow.stop_time = temp_sec + time_table_flow[i].length;
                    flow.number = i;
                    flow_do_control(1);
                    break;
                }
                if(i>=item_number){
                    flow.number = i;
                    flow.stop_time =0;
                }
            }
        }
        if(ligth.stop_time < current_sec){
            item_number = sizeof(time_table_ligth)/sizeof (time_table_t);
            for (u16 i=0;i<item_number;i++){
                temp_sec = time_table_ligth[i].start_time*60;
                if((current_sec >= temp_sec) &&
                    (current_sec <=(temp_sec + time_table_ligth[i].length))){
                    ligth.stop_time = temp_sec + time_table_ligth[i].length;
                    ligth.number = i;
                    ligth_do_control(1);
                    break;
                }
                if(i>=item_number){
                    ligth.number = i;
                    ligth.stop_time =0;
                }
            }
        }
        if(ligth2.stop_time < current_sec){
            item_number = sizeof(time_table_ligth2)/sizeof (time_table_t);
            for (u16 i=0;i<item_number;i++){
                temp_sec = time_table_ligth2[i].start_time*60;
                if((current_sec >= temp_sec) &&
                    (current_sec <=(temp_sec + time_table_ligth2[i].length))){
                    ligth2.stop_time = temp_sec + time_table_ligth2[i].length;
                    ligth2.number = i;
                    ligth2_do_control(1);
                    break;
                }
                if(i>=item_number){
                    ligth2.number = i;
                    ligth2.stop_time =0;
                }
            }
        }
        if(air.stop_time <= current_sec){
            air_do_control(0);
            SSD1306_DrawCircle(8, 33, 7, SSD1306_COLOR_BLACK,1.0);
            SSD1306_UpdateScreen();
        }else{
            u32 pass_time = current_sec - time_table_air[air.number].start_time * 60;
            float part = (float)pass_time/(float)time_table_air[air.number].length;
            SSD1306_DrawCircle(8, 33, 7, SSD1306_COLOR_WHITE,part);
            SSD1306_UpdateScreen();
        }
        if(flow.stop_time <= current_sec){
            flow_do_control(0);
            SSD1306_DrawCircle(23, 33, 7, SSD1306_COLOR_BLACK,1.0);
            SSD1306_UpdateScreen();

        }else{
            u32 pass_time = current_sec - time_table_flow[flow.number].start_time * 60;
            float part = (float)pass_time/(float)time_table_flow[flow.number].length;
            SSD1306_DrawCircle(23, 33, 7, SSD1306_COLOR_WHITE,part);
            SSD1306_UpdateScreen();
        }
        if(ligth.stop_time <= current_sec){
            ligth_do_control(0);
            SSD1306_DrawCircle(38, 33, 7, SSD1306_COLOR_BLACK,1.0);
            SSD1306_UpdateScreen();

        }else{
            u32 pass_time = current_sec - time_table_ligth[ligth.number].start_time * 60;
            float part = (float)pass_time/(float)time_table_ligth[ligth.number].length;
            SSD1306_DrawCircle(38, 33, 7, SSD1306_COLOR_WHITE,part);
            SSD1306_UpdateScreen();

        }
        if(ligth2.stop_time <= current_sec){
            ligth2_do_control(0);
            SSD1306_DrawCircle(53, 33, 7, SSD1306_COLOR_BLACK,1.0);
            SSD1306_UpdateScreen();
        }else{
            u32 pass_time = current_sec - time_table_ligth2[ligth2.number].start_time * 60;
            float part = (float)pass_time/(float)time_table_ligth2[ligth2.number].length;
            SSD1306_DrawCircle(53, 33, 7, SSD1306_COLOR_WHITE,part);
            SSD1306_UpdateScreen();
        }
    }
    return 0x00;
}
void air_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(AIR_PORT,AIR_PIN);
    }else{
        LL_GPIO_ResetOutputPin(AIR_PORT,AIR_PIN);
    }
}
void flow_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(FLOW_PORT,FLOW_PIN);
    }else{
        LL_GPIO_ResetOutputPin(FLOW_PORT,FLOW_PIN);
    }
}
void ligth_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(LIGTH_PORT,LIGTH_PIN);
    }else{
        LL_GPIO_ResetOutputPin(LIGTH_PORT,LIGTH_PIN);
    }
}
void ligth2_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(LIGTH2_PORT,LIGTH2_PIN);
    }else{
        LL_GPIO_ResetOutputPin(LIGTH2_PORT,LIGTH2_PIN);
    }
}

/*@brief set pwm output
 * @param value - [0;100]
 * */
void set_pwm_value(float value){
    u16 pulse = 0;
    static u16 pulse_prev = MAX_PWM_VALUE+1;
    value = value > 100.0f?100.0f:value;
    value = value < 0.0f?0.0f:value;
    value = value/100.0f;
    TIM_OC_InitTypeDef pwm_handle;
    pwm_handle.OCMode = TIM_OCMODE_PWM1;
    if(value == 100.0f){
        pulse = MAX_PWM_VALUE;
    }else if(value == 0.0f){
        pulse = 0;
    }else{
        pulse = (u16)(MAX_PWM_VALUE * value);
    }
    if(pulse_prev != pulse){
        pwm_handle.Pulse = pulse;
        pwm_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
        pwm_handle.OCFastMode = TIM_OCFAST_ENABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_handle, TIM_CHANNEL_1) != HAL_OK){
            _Error_Handler(__FILE__, __LINE__);
        }
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        pulse_prev = pulse;
    }
}

#define IntegralAccum -25
void fb00099_exec(fb00099_IN_type * FBInputs,fb00099_VAR_type * FBVars,\
                  fb00099_OUT_type * FBOutputs) {
    fb00099_IN_type *IN =  FBInputs;
    fb00099_VAR_type *VAR =  FBVars;
    fb00099_OUT_type *OUT = FBOutputs;
    float error_diff; //"невязка" и её изменение
    float du_kp;
    float du_ki;
    float du_kd;
    float du_out;
    if (IN->rezet.data.bit){
        VAR->prev_error_integral.data.float32 = 0.0;
        VAR->error_integral.data.float32  = 0.0;
        VAR->number_tick.data.uint32=0.0;
        VAR->prev_control_integral.data.float32 = IN->position.data.float32;
        IN->rezet.data.bit = 0;
    }
    if (IN->enable.data.bit){
        error_diff = IN->require_value.data.float32 - IN->current_value.data.float32;    //гистерезис не реагирования
        if ((error_diff > -IN->gist_tube.data.float32)&&(error_diff < IN->gist_tube.data.float32)) {
//            error_diff = 0.0;
        }
        if (VAR->number_tick.data.uint32 != 0){
            du_kp = IN->kp.data.float32 * ((error_diff - VAR->error_integral.data.float32));
            du_ki = IN->ki.data.float32 * error_diff;
            if (VAR->number_tick.data.uint32 >= 2){  //добавим дифф составляющию только после 3 такта
                du_kd = IN->kd.data.float32 * ((error_diff - 2*VAR->error_integral.data.float32 + VAR->prev_error_integral.data.float32));
            }else{
                du_kd = 0.0;
            }
        }else{  //при первом расчете расчитываем только Коэфф пропорциональности
            du_kp = IN->kp.data.float32 * (error_diff); //при включении PID не делим на dT
            if (IN->position.data.float32 > 100.0f){VAR->prev_control_integral.data.float32  = 100.0f;}
            if (IN->position.data.float32 < -100.0f){VAR->prev_control_integral.data.float32  = -100.0f;}
            else{ VAR->prev_control_integral.data.float32  = IN->position.data.float32; }
            du_ki = 0.0f;
            du_kd = 0.0f;
        }
        du_out = du_kp + du_ki + du_kd;
        if (du_out > 100.0f) du_out = 100.0f;
        else if (du_out < -100.0f) du_out = -100.0f;
        if (IN->reverse_control.data.bit) du_out = -du_out;
        VAR->prev_control_integral.data.float32 += du_out ;
        VAR->prev_error_integral.data.float32 = VAR->error_integral.data.float32;
        VAR->error_integral.data.float32 = error_diff;
        if (VAR->prev_control_integral.data.float32 > (100.0f + IntegralAccum)) VAR->prev_control_integral.data.float32 = 100.0f + IntegralAccum;
        else if (VAR->prev_control_integral.data.float32 < (-100.0f - IntegralAccum)) VAR->prev_control_integral.data.float32 = -100.0f - IntegralAccum;
        OUT->test.data.float32 = du_out;
        VAR->number_tick.data.uint32++;
    }else{
        VAR->prev_error_integral.data.float32 =  0.0f;
        VAR->error_integral.data.float32  =  0.0f;
        VAR->number_tick.data.uint32= 0.0f;
        if (IN->position.data.float32 > 100.0f){VAR->prev_control_integral.data.float32  = 100.0f;}
        if (IN->position.data.float32 < -100.0f){VAR->prev_control_integral.data.float32  = -100.0f;}
        else{ VAR->prev_control_integral.data.float32  = IN->position.data.float32; }
    }
    //выдаем значение на выход
    OUT->output.data.float32 = VAR->prev_control_integral.data.float32 ;
}


#endif //CONTROL_C
