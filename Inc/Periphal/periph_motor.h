/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_motor.h
 *  Description  : This file contains motor control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:44:03
 *  LastEditTime : 2023-05-05 10:41:51
 */
#ifndef MOTOR_PERIPH_H
#define MOTOR_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_can.h"
#include "util_uart.h"
#include "alg_math.h"
#include "alg_filter.h"

#define MOTOR_GROUP_NUM 4

typedef enum {      
    Motor_TYPE_NOT_CONNECTED    = 0,
    Motor_TYPE_RM3508           = 1,
    Motor_TYPE_RM2006           = 2,
    Motor_TYPE_RM6020           = 3,
} Motor_MotorTypeEnum;  // Motor drive type

typedef struct {
    // Motor feedback
    float angle;
    float speed;
    float current;
    float temp;               // ℃
    uint8_t error_code;

    // Used to calculate continuous angles
    float last_angle;
    int32_t round_count;
    float init_offset;
    float limited_angle;
    float consequent_angle;
} Motor_EncoderTypeDef;

typedef struct _motor_type {
    Motor_EncoderTypeDef encoder;
    Motor_MotorTypeEnum type;

    float output;
    uint32_t id;
    uint8_t is_online;
    uint32_t last_update_time;
    uint8_t init;
    void (*callback)(struct _motor_type*, uint8_t rxbuff[], uint32_t len);
    Filter_LowPassParamTypeDef fdb_fil_param;
    Filter_LowPassTypeDef fdb_fil;
} Motor_MotorTypeDef;

typedef struct {
    uint8_t motor_num;
    Motor_MotorTypeDef* motor_handle[4];
    Motor_MotorTypeEnum type;
    CAN_HandleTypeDef* can_handle;
    CAN_TxHeaderTypeDef can_header;
} Motor_MotorGroupTypeDef;

typedef void (*Motor_EncoderCallbackFuncTypeDef)(Motor_MotorTypeDef*, uint8_t[], uint32_t);

extern Motor_MotorGroupTypeDef Motor_ChassisMotors;
extern Motor_MotorGroupTypeDef Motor_ChassisSteerMotors;
extern Motor_MotorGroupTypeDef Motor_PitchMotors;        
extern Motor_MotorGroupTypeDef Motor_ShootMotors;
extern Motor_MotorGroupTypeDef Motor_FeedMotors;
extern Motor_MotorGroupTypeDef Motor_Big_YawMotors;
extern Motor_MotorGroupTypeDef Motor_Small_YawMotors;


extern Motor_MotorTypeDef Motor_ChassisFontRightMotor;
extern Motor_MotorTypeDef Motor_ChassisFontLeftMotor;
extern Motor_MotorTypeDef Motor_ChassisBackLeftMotor;
extern Motor_MotorTypeDef Motor_ChassisBackRightMotor;
extern Motor_MotorTypeDef Motor_ChassisFontRightSteerMotor;
extern Motor_MotorTypeDef Motor_ChassisFontLeftSteerMotor;
extern Motor_MotorTypeDef Motor_ChassisBackLeftSteerMotor;
extern Motor_MotorTypeDef Motor_ChassisBackRightSteerMotor;
extern Motor_MotorTypeDef Motor_PitchMotor;
extern Motor_MotorTypeDef Motor_Big_YawMotor;   
extern Motor_MotorTypeDef Motor_Small_YawMotor;           

extern Motor_MotorTypeDef Motor_ShootLeftMotor;
extern Motor_MotorTypeDef Motor_ShootRightMotor;
extern Motor_MotorTypeDef Motor_FeedMotor;

/********** VOLATILE USER CODE END **********/


void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
void Motor_InitAllMotors(void);
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                     Motor_EncoderCallbackFuncTypeDef callback);
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, CAN_HandleTypeDef* phcan, uint16_t stdid);
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output);
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor);
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup);
void Motor_SendMotorGroupsOutput(void);
uint8_t Motor_IsAnyMotorOffline(void);
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor);
void rm2006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void gm6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void rm3508_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void rm3508_p19_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
#endif

#ifdef __cplusplus
}
#endif
