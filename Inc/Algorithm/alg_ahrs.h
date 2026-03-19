/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_ahrs.h
 *  Description  : This file contains AHRS algorithm functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-02-11 01:13:19
 */


#ifndef AHRS_ALG_H
#define AHRS_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void AHRS_Init(float quat[4], float ax, float ay, float az, float mx, float my, float mz);
uint8_t AHRS_Update(float quat[4], const float timing_time, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
uint8_t AHRS_UpdateIMU(float quat[4], const float timing_time, float gx, float gy, float gz, float ax, float ay, float az);
void AHRS_GetAngle(float q[4], float *yaw, float *pitch, float *roll);
float AHRS_GetPitch(const float quat[4]);
float AHRS_GetRoll(const float quat[4]);
float AHRS_GetYaw(const float quat[4]);
void AHRS_MahonyUpdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRS_MahonyUpdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
void AHRS_MadgwickUpdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRS_MadgwickUpdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
float AHRS_GetCarrierGravity(void);
void AHRS_AngleToQuat(float quat[4], const float yaw, const float pitch, const float roll);
void AHRS_GetLatitude(float *latit);
void AHRS_GetHeight(float *height);

#ifdef __cplusplus
}
#endif

#endif
