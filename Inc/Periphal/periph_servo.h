/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_servo.h
 *  Description  : This document contains the relevant auxiliary functions of steering gear control
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-05-05 13:25:30
 */


#ifndef SERVO_PERIPH_H
#define SERVO_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif


#include "util_pwm.h"

typedef enum {
    Servo_OFF   = 0,
    Servo_ON    = 1
} Servo_ServoStateEnum;

typedef struct {
    PWM_PWMHandleTypeDef pwm;
    float angle;
    Servo_ServoStateEnum state;
} Servo_ServoTypeDef;


extern Servo_ServoTypeDef Servo_ammoContainerCapServo;

void Servo_InitAllServos(void);
Servo_ServoStateEnum Servo_GetServoState(Servo_ServoTypeDef* servo);
void Servo_StartServo(Servo_ServoTypeDef* servo);
void Servo_StopServo(Servo_ServoTypeDef* servo);
float Servo_GetServoAngle(Servo_ServoTypeDef* servo);
void Servo_SetServoAngle(Servo_ServoTypeDef* servo, float angle);
void Servo_InitServo(Servo_ServoTypeDef* servo, TIM_HandleTypeDef* htim, uint32_t ch);

#endif

#ifdef __cplusplus
}
#endif
