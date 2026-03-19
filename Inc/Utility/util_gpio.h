/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_gpio.h
 *  Description  : This file contains the functions of GPIO
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-05-05 09:38:29
 */


#ifndef GPIO_UTIL_H
#define GPIO_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#define RISE_TRIGGER    0xAA
#define DOWN_TRIGGER    0xBB
#define KEY_FUNC_EVENT_ID   0xA1
#define KEY_BACK_EVENR_ID   0x12

typedef struct {
    GPIO_TypeDef* gpio_handle;
    uint16_t gpio_pin;
    
    uint16_t event_id;

    uint32_t tick;
    uint32_t last_tick;
    GPIO_PinState pin_state;
} GPIO_GPIOTypeDef;


extern GPIO_GPIOTypeDef *CS_ACCEL;
extern GPIO_GPIOTypeDef *CS_GYRO;
extern GPIO_GPIOTypeDef *BMI_INT1;
extern GPIO_GPIOTypeDef *BMI_INT2;
extern GPIO_GPIOTypeDef *LASER;

void GPIO_Set(GPIO_GPIOTypeDef *gpio);
void GPIO_Reset(GPIO_GPIOTypeDef *gpio);
uint32_t GPIO_GetTriggerTick(GPIO_GPIOTypeDef *gpio);
GPIO_PinState GPIO_ReadPin(GPIO_GPIOTypeDef *gpio);

extern osMessageQId Key_QueueHandle;

#ifdef __cplusplus
}
#endif

#endif
