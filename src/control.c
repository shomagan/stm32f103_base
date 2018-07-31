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
extern IWDG_HandleTypeDef hiwdg;
/* ФБ "ПИД" */
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
#define REQUIRE_VALUE 15.0f
static void set_pwm_value(float value);
void control_task( const void *parameters){
    TickType_t control_time;
    static float pid_data = 0.0f;
    u8 tick=0;
    fb00099_IN_type in;
    fb00099_VAR_type var;
    fb00099_OUT_type out;
    var.prev_error_integral.data.float32 = 0.0;
    var.error_integral.data.float32  = 0.0;
    var.number_tick.data.uint32=0.0;
    in.enable.data.bit = 1;				// bit 0 - Ручное, 1 - Автоматическое
    in.reverse_control.data.bit = 1;			// bit 1- реверсивное управление
    in.rezet.data.bit = 0;			// bit 1- сброс накопленных параметров
    in.require_value.data.float32 = REQUIRE_VALUE; 		// float Уставка регулирования
    in.current_value.data.float32 = REQUIRE_VALUE;			// float Регулируемый параметр
    in.kp.data.float32 = 75.0f;		 		// float Коэффициент пропорциональности
    in.ki.data.float32 = 9.0f;		  		// float Коэффициент времени интегрирования
    in.kd.data.float32 = -100.0f;				// float Коэффициент времени интегрирования
    in.position.data.float32 = DEFAULT_OUT;	    	// float - необходимое положение регулятора в процентах
    in.gist_tube.data.float32 = 0.5f;	 		// float Зона нечувствительности в единицах измеряемого параметра
    taskENTER_CRITICAL();
    HAL_IWDG_Refresh(&hiwdg);
    SSD1306_Init();
    HAL_IWDG_Refresh(&hiwdg);
    SSD1306_GotoXY(0, 44); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
    SSD1306_Puts("refregerator", &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
    SSD1306_DrawCircle(10, 33, 7, SSD1306_COLOR_WHITE); //рисуем белую окружность в позиции 10;33 и радиусом 7 пикселей
    SSD1306_UpdateScreen();
    taskEXIT_CRITICAL();
    while(1){
        //ds18_time = osKernelSysTick();
        u8 sensor_data_valid;

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
            SSD1306_Fill(SSD1306_COLOR_BLACK);
            SSD1306_GotoXY(0, 22); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
            sprintf(buff,"temperature %f",in.current_value.data.float32);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
            SSD1306_GotoXY(0, 44); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
            sprintf(buff,"pwm %f",value);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
            SSD1306_UpdateScreen();

        }else{
            char buff[32];
            sprintf(buff,"temperature off");
            SSD1306_Fill(SSD1306_COLOR_BLACK);
            SSD1306_GotoXY(0, 44); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
            SSD1306_UpdateScreen();
            set_pwm_value(DEFAULT_OUT);
            in.position.data.float32 = DEFAULT_OUT;	    	// float - необходимое положение регулятора в процентах
        }
        osDelay(1000);
        HAL_IWDG_Refresh(&hiwdg);
        //osDelayUntil(&ds18_time,_DS18B20_UPDATE_INTERVAL_MS);
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
