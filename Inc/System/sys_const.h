/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.h
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:21:16
 *  LastEditTime : 2023-05-07 10:55:18
 */


#ifndef SYS_CONST_H
#define SYS_CONST_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

extern float Const_SERVO_INIT_OFFSET;


extern const float Const_Shooter15mpers;
extern const float Const_Shooter18mpers;
extern const float Const_Shooter22mpers;
extern const float Const_Shooter30mpers;

extern const float Const_ShooterLockedCurrent;     
extern const float Const_ShooterLockedSpeed;       
extern const float Const_ShooterLockedTime;        
extern const float Const_ShooterRelockedTime;      
extern const float Const_ShooterLockedReverseSpeed;

extern const float Const_FeederSlowSpeed;
extern const float Const_FeederFastSpeed;
extern const float Const_FeederWaitSpeed;

extern const float Const_HeatCtrlFastLimit;  
extern const float Const_HeatCtrlSlowLimit;  
extern const float Const_HeatCtrlWaitLimit;  
extern const float Const_HeatCtrlSingleCount;
extern const float Const_HeatCtrlStopLimit;  

extern const float Const_ShooterSlowSpeed;
extern const float Const_ShooterFastSpeed;

extern const float MOUSE_PITCH_ANGLE_TO_FACT;   
extern float MOUSE_YAW_ANGLE_TO_FACT;     
extern const float MOUSE_CHASSIS_TO_FACT;
extern const float MOUSE_CHASSIS_ACCELERATE;        
extern float MOUSE_CHASSIS_MAX_SPEED;     
extern const float MOUSE_LEG_LEN_DLEN;

extern const float Const_WHEELLEG_REMOTE_YAW_GAIN;
extern const float Const_WHEELLEG_REMOTE_X_GAIN;  
extern const float Const_WHEELLEG_REMOTE_LEN_GAIN;
extern const float Const_Vision_YAW_GAIN;
extern const float Const_Vision_PITCH_GAIN;

extern const float REMOTE_PITCH_ANGLE_TO_REF;

extern const float REMOTE_CHASSIS_VX_GAIN;
extern const float REMOTE_CHASSIS_VY_GAIN;
extern const float REMOTE_CHASSIS_SEP_WZ_GAIN;
extern const float REMOTE_CHASSIS_FOLLOW_WZ_GAIN;
extern const float REMOTE_CHASSIS_FOLLOW_WZ_MAX;
extern const float REMOTE_CHASSIS_XTL_WZ_GAIN;
extern const float CHASSIS_YAW_ANGLE_OFFSET;
extern const float CHASSIS_XTL_WZ;
extern const float CHASSIS_SWERVE_L;   /* 舵轮底盘半长 (m) */
extern const float CHASSIS_SWERVE_W;   /* 舵轮底盘半宽 (m) */
/* 四轮舵向零位偏移 [右后, 左后, 右前, 左前]：该轮物理朝前时编码器角度(deg)，用于安装方向不一致时对齐 */
extern const float Const_ChassisSteerAngleOffset[4];
extern const float Const_Chasiss_Navigate_angular_z ;
extern const float Const_Chasiss_Navigate_linear_x;
extern const float Const_Chasiss_Navigate_linear_y;
extern const float Const_YAW_AUTOADD;
extern const float Const_PITCH_AUTOADD;
extern int   Const_ADD_MAX;

extern float Const_PITCH_UMAXANGLE;         
extern float Const_PITCH_UMAXANGLE_GRYO;    
extern float Const_PITCH_DMAXANGLE;         
extern float Const_YAW_MAXANGLE;            
extern float Const_PITCH_MOTOR_INIT_OFFSETf;
/* 云台大 yaw 零位(deg)：朝前时编码器角度，校准后填入 */
extern const float Const_GimbalBigYawAngleOffset;
/* 云台小 yaw 零位(deg)：锁死目标 = (limited_angle - offset) == 0 */
extern const float Const_GimbalSmallYawAngleOffset;

// DM4310 (Pitch) on CAN2
extern const uint16_t Const_DM4310_PitchMotorId;    // tx StdId = motor_id + mode_id (MIT: +0x000)
extern const uint16_t Const_DM4310_PitchFeedbackId; // rx StdId (mst_id)
extern const uint32_t Const_DM4310_OfflineMs;       // offline threshold for re-enable

extern const float Const_GimbalPitchSpdParam[4][5];
extern const float Const_GimbalPitchAngParam[4][5];
extern const float Const_GimbalYawSpdParam[4][5];
extern const float Const_GimbalYawAngParam[4][5];

extern const float Const_ShootLeftParam[4][5];
extern const float Const_ShootRightParam[4][5];
extern const float Const_FeedAngParam[4][5];
extern const float Const_FeedSpdParam[4][5];

extern const float Const_ChassisFontRightSpdParam[4][5];
extern const float Const_ChassisFontRightAngParam[4][5];
extern const float Const_ChassisFontLeftSpdParam[4][5];
extern const float Const_ChassisFontLeftAngParam[4][5];
extern const float Const_ChassisBackLeftSpdParam[4][5];
extern const float Const_ChassisBackLeftAngParam[4][5];
extern const float Const_ChassisBackRightSpdParam[4][5];
extern const float Const_ChassisBackRightAngParam[4][5];

extern const float QuaternionEKF_F[36];                                   
extern float QuaternionEKF_P[36];

#endif

#ifdef __cplusplus
}
#endif
