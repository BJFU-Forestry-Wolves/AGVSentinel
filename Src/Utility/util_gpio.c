/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_gpio.c
 *  Description  : This file contains the functions of GPIO
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-05-05 11:50:45
 */


#include "util_gpio.h"

GPIO_GPIOTypeDef LASER_START            = {GPIOC, GPIO_PIN_8 , 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CS_ACCEL_START         = {GPIOA, GPIO_PIN_4 , 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CS_GYRO_START          = {GPIOB, GPIO_PIN_0 , 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BMI_INT1_START         = {GPIOC, GPIO_PIN_4 , 0xB1, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BMI_INT2_START         = {GPIOC, GPIO_PIN_5 , 0xB2, 0, GPIO_PIN_RESET};

GPIO_GPIOTypeDef *CS_ACCEL              = &CS_ACCEL_START;
GPIO_GPIOTypeDef *CS_GYRO               = &CS_GYRO_START;
GPIO_GPIOTypeDef *BMI_INT1              = &BMI_INT1_START;
GPIO_GPIOTypeDef *BMI_INT2              = &BMI_INT2_START;
GPIO_GPIOTypeDef *LASER                 = &LASER_START;


/**
 * @brief        : Get the GPIO pin trigger tick
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
uint32_t GPIO_GetTriggerTick(GPIO_GPIOTypeDef *gpio) {
    return gpio->tick;
}


/**
 * @brief        : Set GPIO
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
void GPIO_Set(GPIO_GPIOTypeDef *gpio) {
    gpio->tick = HAL_GetTick();
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_SET);
    gpio->pin_state = GPIO_ReadPin(gpio);
}


/**
 * @brief        : Reset GPIO
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
void GPIO_Reset(GPIO_GPIOTypeDef *gpio) {
    gpio->tick = HAL_GetTick();
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_RESET);
    gpio->pin_state = GPIO_ReadPin(gpio);
}


/**
 * @brief        : Reset GPIO
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
GPIO_PinState GPIO_ReadPin(GPIO_GPIOTypeDef *gpio) {
    gpio->pin_state = HAL_GPIO_ReadPin(gpio->gpio_handle, gpio->gpio_pin);
    return gpio->pin_state;
}
