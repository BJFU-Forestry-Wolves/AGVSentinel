/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.c
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:20:29
 *  LastEditTime : 2023-05-07 10:55:49
 */


#include "sys_const.h"

float Const_SERVO_INIT_OFFSET = 0.053f;

const float Const_Shooter15mpers        = 198.0f;
const float Const_Shooter18mpers        = 220.0f;
const float Const_Shooter22mpers        = 260.0f;
const float Const_Shooter30mpers        = 378.0f;

const float Const_ShooterLockedCurrent              = 3000.0f;
const float Const_ShooterLockedSpeed                = 20.0f;
const float Const_ShooterLockedTime                 = 200.0f;
const float Const_ShooterRelockedTime               = 10.0f;
const float Const_ShooterLockedReverseSpeed         = 0.0f;

const float Const_FeederSlowSpeed                   = 60.0f;
const float Const_FeederFastSpeed                   = 100.0f;
const float Const_FeederWaitSpeed                   = 0.0f;

const float Const_HeatCtrlFastLimit                 = 75;
const float Const_HeatCtrlSlowLimit                 = 40;
const float Const_HeatCtrlWaitLimit                 = 10;
const float Const_HeatCtrlSingleCount               = 10;
const float Const_HeatCtrlStopLimit                 = 10;

const float Const_ShooterSlowSpeed                  = 150.0f;
const float Const_ShooterFastSpeed                  = 225.0f;

const float MOUSE_PITCH_ANGLE_TO_FACT             = 0.008f;
float MOUSE_YAW_ANGLE_TO_FACT               	  = 0.0025f;
const float MOUSE_CHASSIS_TO_FACT                 = 0.00022;       
const float MOUSE_CHASSIS_ACCELERATE              = 0.02f;
float MOUSE_CHASSIS_MAX_SPEED               	  = 12;
const float MOUSE_LEG_LEN_DLEN                    = 0.00018;

const float Const_WHEELLEG_REMOTE_YAW_GAIN              = 0.0005;
const float Const_WHEELLEG_REMOTE_X_GAIN                = 0.000022;
const float Const_WHEELLEG_REMOTE_LEN_GAIN              = 0.00000028;
const float Const_Vision_YAW_GAIN                       = 0.0005;
const float Const_Vision_PITCH_GAIN                     =0.01;

const float REMOTE_PITCH_ANGLE_TO_REF                   = 0.0005f;//0.0005

const float REMOTE_CHASSIS_VX_GAIN                  = 0.5f;
const float REMOTE_CHASSIS_VY_GAIN                  = 0.5f;  
const float REMOTE_CHASSIS_SEP_WZ_GAIN							= -0.25f;
const float REMOTE_CHASSIS_FOLLOW_WZ_GAIN						= 1.0f;
const float REMOTE_CHASSIS_FOLLOW_WZ_MAX						= 175.0f;
const float REMOTE_CHASSIS_XTL_WZ_GAIN							= 1.0f;
const float CHASSIS_YAW_ANGLE_OFFSET								= 125.0f;//：底盘坐标系对齐用的 offset,当车头正对定义的“正前方”时，IMU/云台给出的原始 yaw 角是多少度
const float CHASSIS_XTL_WZ													= 400.0f;//小陀螺转速
const float CHASSIS_SWERVE_L												= 0.282f;  /* 舵轮半长 m，按车体改 */
const float CHASSIS_SWERVE_W												= 0.282f;  /* 舵轮半宽 m，按车体改 */
/* 舵向零位(deg)：轮子物理朝前时该轮 limited_angle 的读数，顺序 [右后, 左后, 右前, 左前]
 * 校准：上电后不动摇杆，用手把四轮都掰到“车体正前方”，用调试读四路 limited_angle 填入下面 */
const float Const_ChassisSteerAngleOffset[4] = {
    135.83f,  /* [0] 右后 BackRight   */
    165.52f,  /* [1] 左后 BackLeft    */
    101.76f,  /* [2] 右前 FontRight   */
    197.08f   /* [3] 左前 FontLeft    */
};
const float Const_Chasiss_Navigate_angular_z        =600.0f;
const float Const_Chasiss_Navigate_linear_x         =300.f;
const float Const_Chasiss_Navigate_linear_y         =300.f;
const float Const_YAW_AUTOADD														= 200.0f;
const float Const_PITCH_AUTOADD													= 5.0f;
int   Const_ADD_MAX																= 2000;

float Const_PITCH_UMAXANGLE                       = 1.92f;//上限位
float Const_PITCH_UMAXANGLE_GRYO                  = 1.92f;  //陀螺模式上限位
float Const_PITCH_DMAXANGLE                       = 1.35f;//下限位
float Const_YAW_MAXANGLE                          = 40.0f; //在普通模式下把 yaw 可偏转范围限制在一个最大角度
float Const_PITCH_MOTOR_INIT_OFFSETf              = 0.0f;

/* 云台大 yaw 轴 6020 零位(deg)：物理朝前时该电机 limited_angle 的读数。
 * 校准：上电后 yaw 不转，用手把云台 yaw 掰到“正前方”，用调试读 Motor_Big_YawMotor.encoder.limited_angle 填入下面 */
const float Const_GimbalBigYawAngleOffset          = 0.0f;
/* 云台小 yaw 零位(deg)：锁死目标 = (limited_angle - offset) == 0 */
const float Const_GimbalSmallYawAngleOffset        = 296.0f;

// DM4310 (Pitch) on CAN2
const uint16_t Const_DM4310_PitchMotorId           = 0x01;
const uint16_t Const_DM4310_PitchFeedbackId        = 0x00;
const uint32_t Const_DM4310_OfflineMs              = 200;

// pitch gimbal param
const float Const_GimbalPitchSpdParam[4][5] = {
    {5.0f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalPitchAngParam[4][5] = {
    {0.5f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
// yaw gimbal param
const float Const_GimbalYawSpdParam[4][5] = {
    {1.0f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};//20

const float Const_GimbalYawAngParam[4][5] = {
    {0.4f, 0, 0, 0, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};//5


// shooter param
const float Const_ShootLeftParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ShootRightParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedAngParam[4][5] = {
    {0.6f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedSpdParam[4][5] = {
    {0.1f, 0, 0, 0, 10.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

// Chassis param（舵向角环 P 降低、加 D，减轻抽搐）
const float Const_ChassisFontRightAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontRightSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontLeftAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontLeftSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1};

float QuaternionEKF_P[36] = {100000,    0.1,    0.1,    0.1,    0.1,    0.1,
                                0.1, 100000,    0.1,    0.1,    0.1,    0.1,
                                0.1,    0.1, 100000,    0.1,    0.1,    0.1,
                                0.1,    0.1,    0.1, 100000,    0.1,    0.1,
                                0.1,    0.1,    0.1,    0.1,    100,    0.1,
                                0.1,    0.1,    0.1,    0.1,    0.1,    100};
