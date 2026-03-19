/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_kalman.h
 *  Description  : This file contains the kalman filter functions (whx designed)
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-23 04:55:25
 */


#ifndef KALMAN_ALG_H
#define KALMAN_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "alg_math.h"

typedef struct kf_t {
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;      // how measurement relates to the state
    float *MeasurementDegree;     // elements of each measurement in H
    float *MatR_DiagonalElements; // variance for each measurement
    float *StateMinVariance;      // suppress filter excessive convergence
    uint8_t *temp;
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5, NonMeasurement;

    mat xhat;      // x(k|k)
    mat xhatminus; // x(k|k-1)
    mat u;         // control vector u
    mat z;         // measurement vector z
    mat P;         // covariance matrix P(k|k)
    mat Pminus;    // covariance matrix P(k|k-1)
    mat F, FT;     // state transition matrix F FT
    mat B;         // control matrix B
    mat H, HT;     // measurement matrix H
    mat Q;         // process noise covariance matrix Q
    mat R;         // measurement noise covariance matrix R
    mat K;         // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} Kalman_KalmanTypeDef;

extern uint16_t sizeof_float, sizeof_double;

void Kalman_FilterInit(Kalman_KalmanTypeDef *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
void Kalman_FilterMeasure(Kalman_KalmanTypeDef *kf);
void Kalman_FilterxhatMinusUpdate(Kalman_KalmanTypeDef *kf);
void Kalman_FilterPminusUpdate(Kalman_KalmanTypeDef *kf);
void Kalman_FilterSetK(Kalman_KalmanTypeDef *kf);
void Kalman_FilterxhatUpdate(Kalman_KalmanTypeDef *kf);
void Kalman_FilterPUpdate(Kalman_KalmanTypeDef *kf);
float *Kalman_FilterUpdate(Kalman_KalmanTypeDef *kf);
static void Kalman_Adjustment_H_K_R(Kalman_KalmanTypeDef *kf);

#ifdef __cplusplus
}
#endif

#endif
