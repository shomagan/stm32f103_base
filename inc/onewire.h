#ifndef ONEWIRE_H
#define ONEWIRE_H
#include "type_def.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#define ONEWIRE_ROM_SIZE 8
#define ONE_WIRE_MAX_DEVICE_NUMBER 5
#define ONE_WIRE_MAX_ERROR_NUM 10

typedef struct{
    GPIO_TypeDef * port;
    uint16_t pin;
    u8 device_rom[ONEWIRE_ROM_SIZE];
    u16 rezerved;
}onewire_device_description_t;
typedef struct{
    float 		temperature;
    uint8_t			data_validate;
}ds18b20sensor_t;
//###################################################################################

extern ds18b20sensor_t	ds18b20[ONE_WIRE_MAX_DEVICE_NUMBER];

extern osThreadId ds18_id;
void ds18_task( const void *parameters)__attribute__ ((noreturn));
#endif //ONEWIRE_H
