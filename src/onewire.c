#ifndef ONEWIRE_C
#define ONEWIRE_C 1
#include "onewire.h"

#include "string.h"
#include "main.h"
#include "stm32f1xx_ll_tim.h"
// Low-level 1-Wire routines
#ifndef DEBUG
#define DEBUG 0
#endif
#define SEARCH_ROM 0xF0
#define MATCH_ROM 0x55
#define SKIP_ROM 0xCC
#define ONEWIRE_CONVERT 0x44
#define RD_SCRATCH 0xBE
#define WR_SCRATCH 0x4E

#define TIMING_RESET1 (480)
#define TIMING_RESET2 (70)
#define TIMING_RESET3 (410)
#define TIMING_READ1 (5)
#define TIMING_READ2 (5)
#define TIMING_READ3 (40)
#define TIMING_WRITE1 (10)
#define TIMING_WRITE2 (50)
#define TIMING_WRITE3 (10)


#define DS18B20_DATA_BUFF_SIZE 9
#define DS18B20_DECIMAL_STEPS_12BIT		0.0625f
#define DS18B20_DECIMAL_STEPS_11BIT		0.125f
#define DS18B20_DECIMAL_STEPS_10BIT		0.25f
#define DS18B20_DECIMAL_STEPS_9BIT		0.5f

static int onewire_bus_reset(onewire_device_description_t * onewire_device_description);
static int onewire_bus_writebit(onewire_device_description_t * onewire_device_description, u8 value);
static u8 onewire_bus_readbit(onewire_device_description_t * onewire_device_description);
int onewire_writebyte(onewire_device_description_t * onewire_device_description, u8 value_in);
static u8 onewire_readbyte(onewire_device_description_t * onewire_device_description);
static int onewire_write(onewire_device_description_t * onewire_device_description, u8 * buff, u16 len);
static int select_rom(onewire_device_description_t * onewire_device_description, u16 len);
static int search_rom(onewire_device_description_t * onewire_device_description, int diff);
static int scan(onewire_device_description_t * onewire_device_description);
static int convert_temp(onewire_device_description_t * onewire_device_description);
static int readinto(onewire_device_description_t * onewire_device_description, u8 * buf, u16 len);
static int read_scratch(onewire_device_description_t * onewire_device_description,u8 * read_buff);
static float read_temp(onewire_device_description_t * onewire_device_description);
static u8 onewire_crc8(u8 * data,u32 data_len);
osThreadId ds18_id;
static u8 device_rom[ONEWIRE_ROM_SIZE*ONE_WIRE_MAX_DEVICE_NUMBER];
static onewire_device_description_t onewire_device_descriptions[ONE_WIRE_MAX_DEVICE_NUMBER];
ds18b20sensor_t	ds18b20[ONE_WIRE_MAX_DEVICE_NUMBER];
#define TIME_YIELD_THRESHOLD 100
void delay_us(uint16_t time_us){
    u16 gap = 0;
    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_EnableCounter(TIM2);
    while(htim2.Instance->CNT <= time_us){
        if(htim2.Instance->CNT > (gap+TIME_YIELD_THRESHOLD)){
            gap = (u16)htim2.Instance->CNT;
            osThreadYield();
        }
    }
    LL_TIM_DisableCounter(TIM2);
    return;
}
void ds18_task( const void *parameters){
    TickType_t ds18_time;
    int device_connected;
    int errors_num = 0;
    (void)parameters;
    for (u8 i=0;i<ONE_WIRE_MAX_DEVICE_NUMBER;i++){
        onewire_device_descriptions[i].pin = DS18B20_PIN;
        onewire_device_descriptions[i].port =  DS18B20_GPIO;
        memset(onewire_device_descriptions[i].device_rom,0,ONEWIRE_ROM_SIZE);
    }
    ds18_time = xTaskGetTickCount();
    device_connected = scan(&onewire_device_descriptions[0]);
    if(device_connected>0){
        //main_printf("onewire devices number - %u\n", device_connected);
    }else{
        //main_printf("onewire devices has not found");
    }

    while(1){
        if (errors_num<ONE_WIRE_MAX_ERROR_NUM && device_connected>0){
            if(convert_temp(&onewire_device_descriptions[0])){
                errors_num++;
            }
            osDelay(750);
            float temp;
            temp = read_temp(&onewire_device_descriptions[0]);
            if(temp > -90.0f){
                ds18b20[0].temperature = temp;
                ds18b20[0].data_validate = 1;
                errors_num = 0;
            }else{
                errors_num++;
            }
#if DEBUG
            main_printf("ds18b20 temp = %f", regs_global.vars.temperature_out);
#endif
        }else{
            ds18b20[0].data_validate = 0;
            osDelay(750);
            device_connected = scan(&onewire_device_descriptions[0]);
            if(device_connected>0){
                errors_num = 0;
            }
        }
    }
}
static float    read_temp(onewire_device_description_t * onewire_device_description){
    u8 read_buff[DS18B20_DATA_BUFF_SIZE];
    float res = -99.0f;
    if (read_scratch(onewire_device_description, read_buff)==0){
        if(onewire_device_description->device_rom[0]==0x10){  //ds18s20
            u8 t = 0;
            if (read_buff[1]){
                t = (read_buff[0] >> 1) | 0x80;
                t = -((~t + 1) & 0xFF);
            }else{
                t = read_buff[0] >> 1;
            }
            s8 t2;
            memcpy(&t2,&t,1);
            return ((float)(t2) - 0.25f + (float)((read_buff[7] - read_buff[6]) / read_buff[7]));
        }else{  //ds18b20
            u8 resolution = ((read_buff[4] & 0x60) >> 5) + 9;
            u8 negative = 0;
#if DEBUG
            main_printf("resolution %u", resolution);
#endif
            u16 blank = (u16)(read_buff[1] << 8) | read_buff[0];
            if (blank & 0x8000){  // sign bit set
                negative = 1;
                blank = ~blank + 1;
            }

            u16 digit = (blank & 0x7ff)/16;
            float decimal = 0.0f;

            switch (resolution) {
                case 9:
                    decimal = (blank >> 3) & 0x01;
                    decimal *= (float)DS18B20_DECIMAL_STEPS_9BIT;
                break;
                case 10:
                    decimal = (blank >> 2) & 0x03;
                    decimal *= (float)DS18B20_DECIMAL_STEPS_10BIT;
                 break;
                case 11:
                    decimal = (blank >> 1) & 0x07;
                    decimal *= (float)DS18B20_DECIMAL_STEPS_11BIT;
                break;
                case 12:
                    decimal = blank & 0x0F;
                    decimal *= (float)DS18B20_DECIMAL_STEPS_12BIT;
                 break;
                default:
                    decimal = 0xFF;
                    digit = 0;
            }
            res = digit + decimal;
            if (negative){  // sign bit set
                res = 0.0f - res;
            }
            return res;
        }
    }else{
        res = -99.0f;
    }
    return res;
}

static int read_scratch(onewire_device_description_t * onewire_device_description,u8 * read_buff){
    int res =0;
    if(onewire_bus_reset(onewire_device_description)){
        select_rom(onewire_device_description, ONEWIRE_ROM_SIZE);
        onewire_writebyte(onewire_device_description, RD_SCRATCH);
        readinto(onewire_device_description, read_buff, DS18B20_DATA_BUFF_SIZE);
        if (onewire_crc8(read_buff, DS18B20_DATA_BUFF_SIZE)){
            res = -2;
        }

    }else{
        res =-1;
    }
    return res;
}

static int convert_temp(onewire_device_description_t * onewire_device_description){
    int res = 0;
    if(onewire_bus_reset(onewire_device_description)){
        onewire_writebyte(onewire_device_description, SKIP_ROM);
        onewire_writebyte(onewire_device_description, ONEWIRE_CONVERT);
    }else{
        res = -1;
    }
    return res;
}

static int scan(onewire_device_description_t * onewire_device_description){
    int devices = 0;
    int diff = 65;
    memset(device_rom,0,ONEWIRE_ROM_SIZE*ONE_WIRE_MAX_DEVICE_NUMBER);
    for (u16 i =0 ;i<ONE_WIRE_MAX_DEVICE_NUMBER;i++){
        diff = search_rom(&onewire_device_description[i], diff);
        if (diff>=0){
            devices += 1;
        }
        if (diff<1){
            break;
        }
    }
    return devices;
}

static int search_rom(onewire_device_description_t * onewire_device_description,int diff){
    int next_diff = 0;
    u8 rom_old[ONEWIRE_ROM_SIZE] = {0};
    for (u8 i =0;i<ONEWIRE_ROM_SIZE;i++){
        rom_old[i] = onewire_device_description->device_rom[i];
    }

    if(onewire_bus_reset(onewire_device_description)){
        onewire_writebyte(onewire_device_description, SEARCH_ROM);
        u8 i = 64;
        for (u8 byte=0;byte<8;byte++){
            u8 r_b;
            r_b = 0;
            for(u8 bit = 0;bit<8;bit++){
                u8 b;
                b = onewire_bus_readbit(onewire_device_description);
                if (onewire_bus_readbit(onewire_device_description)){
                    if (b){  //there are no devices or there is an error on the bus
                        for (u8 i =0;i<ONEWIRE_ROM_SIZE;i++){
                            onewire_device_description->device_rom[i] = 0;
                        }
                        next_diff = -1;
                        return next_diff;
                    }
                }else{
                    if (!b){  // collision, two devices with different bit meaning
                        if ((diff > i) || ((rom_old[byte] & (1 << bit)) && (diff != i))){
                            b = 1;
                            next_diff = i;
                        }
                    }
                }
                onewire_bus_writebit(onewire_device_description, b);
                if (b){
                    r_b |= (1 << bit);
                }
                i -= 1;
            }
            onewire_device_description->device_rom[byte] = r_b;
        }
        return next_diff;
    }else{
        next_diff = -1;
    }
    return next_diff;
}
static int select_rom(onewire_device_description_t * onewire_device_description, u16 len){
    onewire_bus_reset(onewire_device_description);
    onewire_writebyte(onewire_device_description, MATCH_ROM);
    return onewire_write(onewire_device_description, onewire_device_description->device_rom, len);
}
static int readinto(onewire_device_description_t * onewire_device_description, u8 * buf, u16 len){
    int res = 0;
    for (u16 i=0;i<len;i++){
        buf[i] = onewire_readbyte(onewire_device_description);
    }
    return res;
}

static int onewire_write(onewire_device_description_t * onewire_device_description, u8 * buff, u16 len){
    int res = 0;
    for (u16 i=0;i<len;i++){
        res = onewire_writebyte(onewire_device_description, buff[i]);
        if(res<0){
            break;
        }
    }
    return res;
}

static int onewire_bus_reset(onewire_device_description_t * onewire_device_description) {
    int status;
    HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_RESET);
    delay_us(TIMING_RESET1);
    HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_SET);
    delay_us(TIMING_RESET2);
    status = !HAL_GPIO_ReadPin(onewire_device_description->port, onewire_device_description->pin);
    delay_us(TIMING_RESET3);
    return status;
}

static u8 onewire_bus_readbit(onewire_device_description_t * onewire_device_description) {
    u8 value = 0;
    HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_RESET);
    delay_us(TIMING_READ1);
    HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_SET);
    delay_us(TIMING_READ2);
    if (HAL_GPIO_ReadPin(onewire_device_description->port, onewire_device_description->pin)==GPIO_PIN_SET){
        value = 1;
    }
    delay_us(TIMING_READ3);
    return value;
}

static int onewire_bus_writebit(onewire_device_description_t * onewire_device_description, u8 value) {
    int res =0 ;
    HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_RESET);
    delay_us(TIMING_WRITE1);
    if (value & 1) {
        HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_SET);
    }
    delay_us(TIMING_WRITE2);
    HAL_GPIO_WritePin(onewire_device_description->port, onewire_device_description->pin,GPIO_PIN_SET);
    delay_us(TIMING_WRITE3);
    return res;
}

/******************************************************************************/
// MicroPython bindings


static u8 onewire_readbyte(onewire_device_description_t * onewire_device_description) {
    uint8_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= onewire_bus_readbit(onewire_device_description) << i;
    }
    return value;
}

int onewire_writebyte(onewire_device_description_t * onewire_device_description, u8 value_in) {
    int res = 0;
    for (int i = 0; i < 8; ++i) {
        res = onewire_bus_writebit(onewire_device_description, value_in & 1);
        value_in >>= 1;
    }
    return res;
}

static u8 onewire_crc8(u8 * data,u32 data_len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < data_len; ++i) {
        uint8_t byte = data[i];
        for (int b = 0; b < 8; ++b) {
            uint8_t fb_bit = (crc ^ byte) & 0x01;
            if (fb_bit == 0x01) {
                crc = crc ^ 0x18;
            }
            crc = (crc >> 1) & 0x7f;
            if (fb_bit == 0x01) {
                crc = crc | 0x80;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

#endif // ONEWIRE_C
