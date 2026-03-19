/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_kalman.c
 *  Description  : This file contains the kalman filter functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:06
 *  LastEditTime : 2023-05-05 09:23:43
 */


#include "alg_kalman.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"


/**
  * @brief      Initialization of Kalman filter
  * @param      kf :Structure pointer of Kalman filter
  * @param      xhatSize :State variable matrix size
  * @param      uSize :Control matrix size
  * @param      zSize :Observation matrix size
  * @retval     NULL
  */
void Kalman_FilterInit(Kalman_KalmanTypeDef *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize) {
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;
    kf->NonMeasurement = 0;
    
    // measurement flags
    kf->MeasurementMap = (uint8_t *)malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)malloc(sizeof(float) * zSize);
    memset(kf->MeasurementDegree, 0, sizeof(float) * zSize);
    kf->MatR_DiagonalElements = (float *)malloc(sizeof(float) * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof(float) * zSize);
    kf->StateMinVariance = (float *)malloc(sizeof(float) * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof(float) * xhatSize);
    kf->temp = (uint8_t *)malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data
    kf->FilteredValue = (float *)malloc(sizeof(float) * xhatSize);
    memset(kf->FilteredValue, 0, sizeof(float) * xhatSize);
    kf->MeasuredVector = (float *)malloc(sizeof(float) * zSize);
    memset(kf->MeasuredVector, 0, sizeof(float) * zSize);
    kf->ControlVector = (float *)malloc(sizeof(float) * uSize);
    memset(kf->ControlVector, 0, sizeof(float) * uSize);

    // xhat x(k|k)
    kf->xhat_data = (float *)malloc(sizeof(float) * xhatSize);
    memset(kf->xhat_data, 0, sizeof(float) * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus x(k|k-1)
    kf->xhatminus_data = (float *)malloc(sizeof(float) * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof(float) * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0) {
        // control vector u
        kf->u_data = (float *)malloc(sizeof(float) * uSize);
        memset(kf->u_data, 0, sizeof(float) * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z
    kf->z_data = (float *)malloc(sizeof(float) * zSize);
    memset(kf->z_data, 0, sizeof(float) * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k)
    kf->P_data = (float *)malloc(sizeof(float) * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof(float) * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    //create covariance matrix P(k|k-1)
    kf->Pminus_data = (float *)malloc(sizeof(float) * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof(float) * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F FT
    kf->F_data = (float *)malloc(sizeof(float) * xhatSize * xhatSize);
    kf->FT_data = (float *)malloc(sizeof(float) * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof(float) * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof(float) * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0) {
        // control matrix B
        kf->B_data = (float *)malloc(sizeof(float) * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof(float) * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    // measurement matrix H
    kf->H_data = (float *)malloc(sizeof(float) * zSize * xhatSize);
    kf->HT_data = (float *)malloc(sizeof(float) * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof(float) * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof(float) * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q
    kf->Q_data = (float *)malloc(sizeof(float) * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof(float) * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R
    kf->R_data = (float *)malloc(sizeof(float) * zSize * zSize);
    memset(kf->R_data, 0, sizeof(float) * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K
    kf->K_data = (float *)malloc(sizeof(float) * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof(float) * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)malloc(sizeof(float) * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)malloc(sizeof(float) * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)malloc(sizeof(float) * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)malloc(sizeof(float) * kf->xhatSize);
    kf->temp_vector_data1 = (float *)malloc(sizeof(float) * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}


/** 
  * @brief      Matrix H K R auto adjustment
  * @param      *kf ：
  * @retval        
 */
void Kalman_FilterMeasure(Kalman_KalmanTypeDef *kf) {
    if (kf->UseAutoAdjustment != 0)
        Kalman_Adjustment_H_K_R(kf);
    else {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof(float) * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof(float) * kf->uSize);
}


/** 
  * @brief         Kalman Filter xhat Minus Update
  * @param *kf
  * @retval        
 */
void Kalman_FilterxhatMinusUpdate(Kalman_KalmanTypeDef *kf) {
    if (!kf->SkipEq1) {
        if (kf->uSize > 0) {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}


/** 
  * @brief         Kalman Filter Pminus Update
  * @param *kf
  * @retval        
 */
void Kalman_FilterPminusUpdate(Kalman_KalmanTypeDef *kf) {
    if (!kf->SkipEq2) {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}


/** 
  * @brief         Kalman Filter Set K
  * @param *kf
  * @retval        
 */
void Kalman_FilterSetK(Kalman_KalmanTypeDef *kf) {
    if (!kf->SkipEq3) {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}


/** 
  * @brief         Kalman Filter xhat Update
  * @param *kf
  * @retval        
 */
void Kalman_FilterxhatUpdate(Kalman_KalmanTypeDef *kf) {
    if (!kf->SkipEq4) {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}


/** 
  * @brief         Kalman Filter P Update
  * @param *kf
  * @retval        
 */
void Kalman_FilterPUpdate(Kalman_KalmanTypeDef *kf) {
    if (!kf->SkipEq5) {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}


/**
  * @brief      Update data with Kalman filter
  * @param      kf :Structure pointer of Kalman filter
  * @retval     Filtered data pointer
  */
float *Kalman_FilterUpdate(Kalman_KalmanTypeDef *kf) {

    Kalman_FilterMeasure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_FilterxhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_FilterPminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0) {
        // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
        Kalman_FilterSetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        Kalman_FilterxhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        Kalman_FilterPUpdate(kf);
    }
    else {
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof(float) * kf->xhatSize * kf->xhatSize);
    }

    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; i++) {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof(float) * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}


/**
  * @brief      Auto adjust H K R matrix
  * @param      kf :Structure pointer of Kalman filter
  * @retval     NULL
  */
static void Kalman_Adjustment_H_K_R(Kalman_KalmanTypeDef *kf) {

    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof(float) * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);

    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof(float) * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof(float) * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; i++) {
        if (kf->z_data[i] != 0) {
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++) {

        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
