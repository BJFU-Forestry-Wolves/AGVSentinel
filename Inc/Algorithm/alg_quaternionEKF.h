/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : alg_quaternionEKF.h
 *  Description  : attitude update with gyro bias estimate and chi-square test
 *  LastEditors  : Polaris
 *  Date         : 2023-02-06 00:50:53
 *  LastEditTime : 2023-02-10 15:51:20
 */


#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "alg_kalman.h"

#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct {
    uint8_t Initialized;
    Kalman_KalmanTypeDef QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        
    float GyroBias[3]; 

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; 
    float Q2; 
    float R;  

    float dt; 
    mat ChiSquare;
    float ChiSquare_Data[1];      
    float ChiSquareTestThreshold; 
    float lambda;                 

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static void QuaternionEKF_Observe(Kalman_KalmanTypeDef *kf);
static void QuaternionEKF_F_Linearization_P_Fading(Kalman_KalmanTypeDef *kf);
static void QuaternionEKF_SetH(Kalman_KalmanTypeDef *kf);
static void QuaternionEKF_xhatUpdate(Kalman_KalmanTypeDef *kf);

#endif

#ifdef __cplusplus
}
#endif
