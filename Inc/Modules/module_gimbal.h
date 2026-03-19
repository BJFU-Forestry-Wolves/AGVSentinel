/*
 *  Project      : Polaris
 * 
 *  file         : cha_gimbal_ctrl.h
 *  Description  : This file contains Gimbal control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 11:15:17
 */

#ifndef MODULE_GIMBAL_H
#define MODULE_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "periph_motor.h"
#include "alg_math.h"
#include "alg_pid.h"

typedef struct {
    float pitch_ref;                                  // 云台Pitch轴角度目标值
    uint8_t pitch_ref_limit_status;                   // Gimbal Pitch limit status 
    uint8_t pitch_count;

    uint8_t control_state;                          // Whether to enable control 1 Yes 0 No 
    uint8_t output_state;                           // Whether to enable output 1 Yes 0 No 
    uint8_t pending_state;                          // Gimbal Pitch Occupy Lock 1 Yes 0 No 

    PID_PIDTypeDef spdPID;
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;

} GimbalPitch_GimbalPitchTypeDef;

typedef struct {
    float yaw_ref;                                  // Gimbal Yaw angle target value 
    float yaw_position_fdb;                         // Gimbal Yaw IMU angle feedback value 
    float yaw_speed_fdb;                            // Gimbal Yaw IMU angular velocity feedback value
    uint8_t yaw_ref_limit_status;                   // Gimbal Yaw limit status 
    uint8_t yaw_count;

    uint8_t control_state;                          // Whether to enable control 1 Yes 0 No 
    uint8_t output_state;                           // Whether to enable output 1 Yes 0 No 
    uint8_t pending_state;                          // Gimbal Yaw Occupy Lock 1 Yes 0 No 

    PID_PIDTypeDef spdPID;
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;
	
	PID_PIDTypeDef GimbalSmallYaw_spdPID;
	PID_PIDParamTypeDef GimbalSmallYaw_spdPIDParam;
	PID_PIDTypeDef GimbalSmallYaw_angPID;
	PID_PIDParamTypeDef GimbalSmallYaw_angPIDParam;
	
	

} GimbalYaw_GimbalYawTypeDef;



extern GimbalPitch_GimbalPitchTypeDef GimbalPitch_gimbalPitchControlData;
extern GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;


void GimbalPitch_InitGimbalPitch(void);
GimbalPitch_GimbalPitchTypeDef* GimbalPitch_GetGimbalPitchPtr(void);
void GimbalPitch_SetGimbalPitchControlState(uint8_t state);
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state);
void GimbalPitch_SetPitchRef(float pitch_ref);
float Gimbal_LimitPitch(float ref);
float Gimbal_LimitYaw(float ref);
void GimbalPitch_Control(void);
void GimbalPitch_Output(void);


void GimbalYaw_InitGimbalYaw(void);
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr(void);
void GimbalYaw_SetGimbalYawControlState(uint8_t state);
void GimbalYaw_SetGimbalYawOutputState(uint8_t state);
void GimbalYaw_SetYawRef(float yaw_ref);
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb);
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb);
void GimbalYaw_Control(void);
void GimbalYaw_Output(void);
/** 校准：读大 yaw 编码器原始角(0~360)，朝前时填入 Const_GimbalBigYawAngleOffset */
float GimbalYaw_GetEncoderRawDeg(void);
/** 校准：逻辑角 = 编码器 - offset，填好 offset 后朝前为 0° */
float GimbalYaw_GetEncoderLogicalDeg(void);

#endif

#ifdef __cplusplus
}
#endif
