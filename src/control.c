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
#include "cmsis_os.h"
#include "onewire.h"
#include "ssd1306.h"
#include "time_table.h"
#include "stm32f1xx_ll_gpio.h"
#include "main.h"
#define USE_CONST_PWM 0
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
} pid_in_type;

typedef struct {
    register_type error_integral;		// float - накопленная ошибка интегратора
    register_type prev_error_integral;			// float - предыдущее значение ошибки регулирования
    register_type prev_control_integral;			// float - накопленное воздействия на регулирующий орган
    register_type enable_old;			// bit - для отслеживания первого такта включения
    register_type number_tick;			// uint32 - количество тактов после включения,для интервала работы
} pid_var_type;

typedef struct {
    register_type error;	     	// bit Индикация ошибки входных параметров
    register_type output;	    	// float - необходимое положение регулятора в процентах
    register_type test;				// float
} pid_out_type;

void pid_exec(pid_in_type * FBInputs,pid_var_type * FBVars,\
                  pid_out_type * FBOutputs);

#define DEFAULT_OUT 0.0f
#define REQUIRE_VALUE -40.0f
static void set_pwm_value(float value);
static void set_controller_value(float value);
static u8 check_state_machine(void);
static void air_do_control(u8 enable);
static void flow_do_control(u8 enable);
static void ligth1_do_control(u8 enable);
static void ligth2_do_control(u8 enable);
static void set_pwm_value_const(float value);


typedef enum{
    SOFT_DISABLE,
    SOFT_AWAKE,
    SOFT_SLEEP,
    SOFT_SUSPEND,
}soft_state_t;
static void soft_state_control(soft_state_t command);
static soft_state_t soft_state = SOFT_DISABLE;
const static u16 SOFT_TIME_INTERVAL_MS = 20000;
static u32 soft_time_start = 0;
static u16 soft_time_interval = 0;
static osThreadId soft_handle_task_id;

static void soft_handle_task( const void *parameters);

typedef struct __attribute__((__packed__)){
    u16 number;
    u32 stop_time;  //in second
}state_machine;

static const time_table_t time_table_ligth1[] = {
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
    {600,600},
    {660,600},
    {720,600},
    {780,600},
    {840,600},
    {900,600},
    {960,600},
    {1020,600},
    {1080,600},
    {1140,600},
};

static const time_table_t time_table_flow[] = {
    {630,900},
    {690,900},
    {750,900},
    {810,900},
    {870,900},
    {930,900},
    {990,900},
    {1050,900},
    {1110,900},
    {1170,900},
};

static state_machine flow,ligth1,ligth2,air;
void control_task( const void *parameters){
    u32 tick=0;
    u32 time;
#if CABLE_HEATING==0
    pid_in_type in;
    pid_var_type var;
    pid_out_type out;
    var.prev_error_integral.data.float32 = 0.0;
    var.error_integral.data.float32  = 0.0;
    var.number_tick.data.uint32=0.0;
    in.enable.data.bit = 1;
    in.reverse_control.data.bit = 1;//set reverse control
    in.rezet.data.bit = 0;
    in.require_value.data.float32 = REQUIRE_VALUE;
    in.current_value.data.float32 = REQUIRE_VALUE;
    in.kp.data.float32 = 75.0f;
    in.ki.data.float32 = 9.0f;
    in.kd.data.float32 = -100.0f;
    in.position.data.float32 = DEFAULT_OUT;
    in.gist_tube.data.float32 = 0.5f;
#endif
    HAL_IWDG_Refresh(&hiwdg);
    HAL_IWDG_Refresh(&hiwdg);
    flow.stop_time = 0;
    ligth1.stop_time = 0;
    ligth2.stop_time = 0;
    air.stop_time = 0;
    osThreadDef(soft_handle_task, soft_handle_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    soft_handle_task_id = osThreadCreate(osThread(soft_handle_task), NULL);
    osDelay(1);
    time = osKernelSysTick();
    SSD1306_Init();
    SSD1306_UpdateScreen();
#if CABLE_HEATING
    u8 enabled = 0;
    u8 first_start=0;
#endif
    while(1){
        //ds18_time = osKernelSysTick();
        u8 sensor_data_valid;
        tick++;
        sensor_data_valid = 0;
#if CABLE_HEATING==0
        if(ds18b20[0].data_validate) {
            in.current_value.data.float32  = ds18b20[0].temperature;
        }
        pid_exec(&in,&var,&out);
#else

#endif
        if(SSD1306.error_num){
            SSD1306.Initialized = 0;
        }
        if((tick % 5) == 0){
            if(!SSD1306.Initialized ){
                SSD1306_Init();
            }
        }
        if(ds18b20[0].data_validate) {
            char buff[32] = "temp - ";
            buff[9] = 0;
#if CABLE_HEATING==0
            SSD1306_GotoXY(0, 44);
#else
            SSD1306_GotoXY(0, 15);
#endif
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            int temp1,temp2,min_temp;
            temp2 = -0;
#if CABLE_HEATING==0
            static float value;
            value = out.output.data.float32 >= 0.0f?out.output.data.float32:0.0f;
            set_controller_value(value);
            in.position.data.float32 = out.output.data.float32;
            temp = (int)in.current_value.data.float32;
#else


            temp1 = ds18b20[0].temperature;
            min_temp = temp1;
            if(ds18b20[1].data_validate == 1){
                temp2 = ds18b20[1].temperature;
                if (temp2<min_temp){
                    min_temp = temp2;
                }
            }
            if(!enabled){
                if (min_temp<=(stored_data.stored_struct.temperature_min)){
                    enabled = 1;
                    LL_GPIO_SetOutputPin(PID_OUT_PORT_0, PID_OUT_PIN_LL_0);
                    stored_data.stored_struct.trigers_num++;
                    save_stored_struct(&stored_data);
                }
            }else{
                if (min_temp>(stored_data.stored_struct.temperature_min+stored_data.stored_struct.temperature_triger)){
                    enabled = 0;
                    LL_GPIO_ResetOutputPin(PID_OUT_PORT_0, PID_OUT_PIN_LL_0);
                }
            }
            if((osKernelSysTick()<(1000*60)) && (first_start<2)){
                if(first_start == 0){
                    stored_data.stored_struct.trigers_num++;
                    save_stored_struct(&stored_data);
                    first_start = 1;
                }
                LL_GPIO_SetOutputPin(PID_OUT_PORT_0, PID_OUT_PIN_LL_0);
            }else{
                if(first_start==1){
                    if(!enabled){
                        LL_GPIO_ResetOutputPin(PID_OUT_PORT_0, PID_OUT_PIN_LL_0);
                    }
                    first_start = 2;
                }
            }
#endif
            itoa(temp1,buff,10);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_Puts(" ", &Font_7x10, SSD1306_COLOR_WHITE);
            itoa(temp2,buff,10);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            buff[0] = ' ';buff[1] = 'C';buff[2] = ' ';buff[3] = ' ';buff[4] = ' ';
            buff[5] = ' ';
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
#if CABLE_HEATING
            SSD1306_GotoXY(0, 30);
            if (enabled){
                SSD1306_Puts("heater on   ", &Font_7x10, SSD1306_COLOR_WHITE);
            }else if(first_start==1){
                SSD1306_Puts("heater test ", &Font_7x10, SSD1306_COLOR_WHITE);
            }else{
                SSD1306_Puts("heater off  ", &Font_7x10, SSD1306_COLOR_WHITE);
            }
            SSD1306_GotoXY(0, 45);
            SSD1306_Puts("trigers", &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_GotoXY(64, 45);
            itoa(stored_data.stored_struct.trigers_num,buff,10);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_Puts("   ", &Font_7x10, SSD1306_COLOR_WHITE);
#endif
            SSD1306_UpdateScreen();
        }else{

            char buff[] = "temperature off";
#if CABLE_HEATING==0
            SSD1306_GotoXY(0, 44);
#else
            SSD1306_GotoXY(0, 15);
            LL_GPIO_ResetOutputPin(PID_OUT_PORT_0, PID_OUT_PIN_LL_0);
            enabled = 0;
#endif
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
            SSD1306_UpdateScreen();
            set_controller_value(DEFAULT_OUT);
#if CABLE_HEATING==0
            in.position.data.float32 = DEFAULT_OUT;	    	// float - необходимое положение регулятора в процентах
#endif
        }
        RTC_TimeTypeDef rtc_time;
        HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);
        SSD1306_GotoXY(0, 0); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
#if CABLE_HEATING==0
        char buff[32] = "time ";
        buff[7] = 0;
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
        itoa(rtc_time.Hours,buff,10);
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
        buff[0] = 0x3a ;// ":"
        buff[1] = 0 ;// ":"
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
        itoa(rtc_time.Minutes,buff,10);
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
        buff[0] = 0x3a ;// ":"
        buff[1] = 0 ;// ":"
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
        itoa(rtc_time.Seconds,buff,10);
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
        for (u8 i=0;i<5;i++){
            buff[i] = ' ';
        }
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
#else
        char buff[32] = "min ";
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
        itoa(stored_data.stored_struct.temperature_min,buff,10);
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
        memcpy(buff," trig ",6);
        buff[6] = 0;
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
        itoa(stored_data.stored_struct.temperature_triger,buff,10);
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
        for (u8 i=0;i<5;i++){
            buff[i] = ' ';
        }
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
#endif
        SSD1306_UpdateScreen();
#if CABLE_HEATING==0
        check_state_machine();
#endif
        HAL_IWDG_Refresh(&hiwdg);
        set_pwm_value_const(40.0);
        osDelayUntil((uint32_t*)&time,1000);
    }
}
u8 check_state_machine(){
    u16 item_number;
    u32 temp_sec;
    u16 i;
    RTC_TimeTypeDef time;
    HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
    if((time.Hours==0) && (time.Minutes==0)){
        flow.stop_time = 0;
        ligth1.stop_time = 0;
        ligth2.stop_time = 0;
        air.stop_time = 0;
        air_do_control(0);
        flow_do_control(0);
        ligth1_do_control(0);
        ligth2_do_control(0);
        soft_state_control(SOFT_DISABLE);
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
        if(ligth1.stop_time < current_sec){
            item_number = sizeof(time_table_ligth1)/sizeof (time_table_t);
            for (u16 i=0;i<item_number;i++){
                temp_sec = time_table_ligth1[i].start_time*60;
                if((current_sec >= temp_sec) &&
                    (current_sec <=(temp_sec + time_table_ligth1[i].length))){
                    ligth1.stop_time = temp_sec + time_table_ligth1[i].length;
                    ligth1.number = i;
                    ligth1_do_control(1);
                    break;
                }
                if(i>=item_number){
                    ligth1.number = i;
                    ligth1.stop_time =0;
                }
            }
        }
        if(ligth2.stop_time <= current_sec){
            item_number = sizeof(time_table_ligth2)/sizeof (time_table_t);
            for (u16 i=0;i<item_number;i++){
                temp_sec = time_table_ligth2[i].start_time*60;
                if((current_sec >= temp_sec) &&
                    (current_sec <(temp_sec + SOFT_TIME_INTERVAL_MS/1000))){
                    ligth2.stop_time = temp_sec + SOFT_TIME_INTERVAL_MS/1000;
                    ligth2.number = i;
                    ligth2_do_control(1);
                    soft_state_control(SOFT_AWAKE);
                    break;
                }else if((current_sec >= (temp_sec + SOFT_TIME_INTERVAL_MS/1000)) &&
                         (current_sec <(temp_sec + time_table_ligth2[i].length - SOFT_TIME_INTERVAL_MS/1000))){
                    ligth2.stop_time = temp_sec + time_table_ligth2[i].length - SOFT_TIME_INTERVAL_MS/1000;
                    ligth2.number = i;
                    ligth2_do_control(1);
                    set_pwm_value(100.0);
                    soft_state_control(SOFT_SUSPEND);
                }else if((current_sec >= (temp_sec + time_table_ligth2[i].length - SOFT_TIME_INTERVAL_MS/1000)) &&
                         (current_sec <(temp_sec + time_table_ligth2[i].length))){
                    ligth2.stop_time = temp_sec + time_table_ligth2[i].length;
                    ligth2.number = i;
                    ligth2_do_control(1);
                    soft_state_control(SOFT_SLEEP);
                }
                if(i>=item_number){
                    ligth2.number = i;
                    ligth2.stop_time = 0;
                    soft_state_control(SOFT_DISABLE);
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
            SSD1306_DrawCircle(35, 33, 7, SSD1306_COLOR_BLACK,1.0);
            SSD1306_UpdateScreen();

        }else{
            u32 pass_time = current_sec - time_table_flow[flow.number].start_time * 60;
            float part = (float)pass_time/(float)time_table_flow[flow.number].length;
            SSD1306_DrawCircle(35, 33, 7, SSD1306_COLOR_WHITE,part);
            SSD1306_UpdateScreen();
        }
        if(ligth1.stop_time <= current_sec){
            ligth1_do_control(0);
            SSD1306_DrawCircle(70, 33, 7, SSD1306_COLOR_BLACK,1.0);
            SSD1306_UpdateScreen();

        }else{
            u32 pass_time = current_sec - time_table_ligth1[ligth1.number].start_time * 60;
            float part = (float)pass_time/(float)time_table_ligth1[ligth1.number].length;
            SSD1306_DrawCircle(70, 33, 7, SSD1306_COLOR_WHITE,part);
            SSD1306_UpdateScreen();

        }
        if(ligth2.stop_time < current_sec){
            ligth2_do_control(0);
            soft_state_control(SOFT_DISABLE);
            SSD1306_DrawCircle(98, 33, 7, SSD1306_COLOR_BLACK,1.0);
            SSD1306_UpdateScreen();
        }else{
            u32 pass_time = current_sec - time_table_ligth2[ligth2.number].start_time * 60;
            float part = (float)pass_time/(float)time_table_ligth2[ligth2.number].length;
            SSD1306_DrawCircle(98, 33, 7, SSD1306_COLOR_WHITE,part);
            SSD1306_UpdateScreen();
        }
        SSD1306_GotoXY(0, 12);
        SSD1306_Puts("air", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(28, 12);
        SSD1306_Puts("flow", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(63, 12);
        SSD1306_Puts("led", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(91, 12);
        SSD1306_Puts("led", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();

    }

    return 0x00;
}
static void air_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(AIR_PORT,AIR_PIN);
    }else{
        LL_GPIO_ResetOutputPin(AIR_PORT,AIR_PIN);
    }
}
static void flow_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(FLOW_PORT,FLOW_PIN);
    }else{
        LL_GPIO_ResetOutputPin(FLOW_PORT,FLOW_PIN);
    }
}
static void ligth1_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(LIGTH1_PORT,LIGTH1_PIN);
    }else{
        LL_GPIO_ResetOutputPin(LIGTH1_PORT,LIGTH1_PIN);
    }
}
static void ligth2_do_control(u8 enable){
    if(enable){
        LL_GPIO_SetOutputPin(LIGTH2_PORT,LIGTH2_PIN);
    }else{
        LL_GPIO_ResetOutputPin(LIGTH2_PORT,LIGTH2_PIN);
    }
}

/**
 * @brief set pwm output
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
    if(value >= 0.99f){
        pulse = MAX_PWM_VALUE;
    }else if(value <= 0.01f){
        pulse = 0;
    }else{
        pulse = (u16)(MAX_PWM_VALUE * value);
    }
    if(pulse_prev != pulse){
#if USE_CONST_PWM==0
        pwm_handle.Pulse = pulse;
        pwm_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
        pwm_handle.OCFastMode = TIM_OCFAST_ENABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_handle, TIM_CHANNEL_1) != HAL_OK){
            _Error_Handler(__FILE__, __LINE__);
        }
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
#endif

    }
}
/**
 * @brief set pwm output
 * @param value - [0;100]
 * */
static void set_pwm_value_const(float value){
    u16 pulse = 0;
    static u16 pulse_prev = MAX_PWM_VALUE+1;
    value = value > 100.0f?100.0f:value;
    value = value < 0.0f?0.0f:value;
    value = value/100.0f;
    TIM_OC_InitTypeDef pwm_handle;
    pwm_handle.OCMode = TIM_OCMODE_PWM1;
    if(value >= 0.99f){
        pulse = MAX_PWM_VALUE;
    }else if(value <= 0.01f){
        pulse = 0;
    }else{
        pulse = (u16)(MAX_PWM_VALUE * value);
    }
    if(pulse_prev != pulse){
#if USE_CONST_PWM
        pwm_handle.Pulse = pulse;
        pwm_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
        pwm_handle.OCFastMode = TIM_OCFAST_ENABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_handle, TIM_CHANNEL_1) != HAL_OK){
            _Error_Handler(__FILE__, __LINE__);
        }
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
#endif
        pulse_prev = pulse;
    }
}
/**
 * @brief set pwm output
 * @param value - [0;100]
 * */
void set_controller_value(float value){
    if(value > 0.0f){
        LL_GPIO_SetOutputPin(PID_OUT_PORT_3, PID_OUT_PIN_3);
    }else{
        LL_GPIO_ResetOutputPin(PID_OUT_PORT_3, PID_OUT_PIN_3);
    }
}
#define IntegralAccum -25
void pid_exec(pid_in_type * FBInputs,pid_var_type * FBVars,\
                  pid_out_type * FBOutputs) {
    pid_in_type *IN =  FBInputs;
    pid_var_type *VAR =  FBVars;
    pid_out_type *OUT = FBOutputs;
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

void soft_state_control(soft_state_t command){
    u32 current_time = osKernelSysTick();
    if(soft_state != command){
        soft_state = command;
        soft_time_start = current_time;
    }
    if ((current_time-soft_time_start)<SOFT_TIME_INTERVAL_MS){
        float value_pwm;
        switch(soft_state){
        case SOFT_DISABLE:
            set_pwm_value(0.0);
            if(osThreadIsSuspended(soft_handle_task_id)!=osOK){
                osThreadSuspend(soft_handle_task_id);
            }
            break;
        case SOFT_SUSPEND:
            if(osThreadIsSuspended(soft_handle_task_id)!=osOK){
                osThreadSuspend(soft_handle_task_id);
            }
            break;
        case SOFT_AWAKE:
            value_pwm = (((current_time-soft_time_start)*100)/SOFT_TIME_INTERVAL_MS);
            set_pwm_value(value_pwm);
            if(osThreadIsSuspended(soft_handle_task_id)==osOK){
                osThreadResume(soft_handle_task_id);
            }
            break;
        case SOFT_SLEEP:
            value_pwm = (((SOFT_TIME_INTERVAL_MS-(current_time-soft_time_start))*100)/SOFT_TIME_INTERVAL_MS);
            set_pwm_value(value_pwm);
            if(osThreadIsSuspended(soft_handle_task_id)==osOK){
                osThreadResume(soft_handle_task_id);
            }
            break;
        }
    }
}
static void soft_handle_task( const void *parameters){
    uint32_t task_time = osKernelSysTick();
    osThreadId thread_id;
    thread_id = osThreadGetId();
    osThreadSuspend(thread_id);
    while(1){
        u32 current_time = osKernelSysTick();
        if ((current_time-soft_time_start)<SOFT_TIME_INTERVAL_MS){
            float value_pwm;
            switch(soft_state){
            case SOFT_DISABLE:
                set_pwm_value(0.0);
                break;
            case SOFT_AWAKE:
                value_pwm = (((current_time-soft_time_start)*100)/SOFT_TIME_INTERVAL_MS);
                set_pwm_value(value_pwm);
                break;
            case SOFT_SLEEP:
                value_pwm = (((SOFT_TIME_INTERVAL_MS-(current_time-soft_time_start))*100)/SOFT_TIME_INTERVAL_MS);
                set_pwm_value(value_pwm);
                break;
            }
        }
        osDelayUntil(&task_time,1);
    }
}
#endif //CONTROL_C
