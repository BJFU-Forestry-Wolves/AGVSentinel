/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_ins.h
 *  Description  : Attitude solution
 *  LastEditors  : Polaris
 *  Date         : 2023-01-24 01:43:43
 *  LastEditTime : 2023-03-09 10:03:20
 */


#ifndef INS_TASK_H
#define INS_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define X_INS 0
#define Y_INS 1
#define Z_INS 2

#define INS_TASK_PERIOD 1

typedef struct {
    float q[4]; //四元数（Quaternion），数组长度 4，存储 q[0] (w)、q[1] (x)、q[2] (y)、q[3] (z)
    float Gyro[3];  //存储陀螺仪的三轴原始数据，对应 X/Y/Z 轴
    float Accel[3]; //存储加速度计的三轴原始数据
    float MotionAccel_b[3]; //车身坐标系数据
    float MotionAccel_n[3]; //世界坐标系数据

    float AccelLPF; 

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;
	/*计算得到的欧拉角:横滚角(Roll),俯仰角(Pitch),偏航角(Yaw)*/
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_INSTypeDef;

typedef struct {
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} INS_DataTypeDef;

extern INS_INSTypeDef INS;

void INS_Init(void);
INS_INSTypeDef *INS_GetINSPtr(void);
void Ins_Task(void const * argument);
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void Param_Correction(INS_DataTypeDef *param, float gyro[3], float accel[3]);

#endif
