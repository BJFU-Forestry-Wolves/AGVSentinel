/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_math.h
 *  Description  : This file contains the math calculate tools
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-04-02 10:17:22
 */


#ifndef MATH_ALG_H
#define MATH_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "arm_math.h"

#define mat                 arm_matrix_instance_f32
#define Matrix_Init         arm_mat_init_f32
#define Matrix_Add          arm_mat_add_f32
#define Matrix_Subtract     arm_mat_sub_f32
#define Matrix_Multiply     arm_mat_mult_f32
#define Matrix_Transpose    arm_mat_trans_f32
#define Matrix_Inverse      arm_mat_inverse_f32

/**
  * @brief      Limit function
  * @param      input :Limited value
  * @param      max :Max limite value
  * @retval     NULL
  */
#define LimitMax(input, max) {      \
        if (input > max) {          \
            input = max;            \
        }                           \
        else if (input < -max) {    \
            input = -max;           \
        }                           \
    }


/**
  * @brief      Maximum and minimum limiting
  * @param      input :Limited value
  * @param      max :Max limite value
  * @param      min :Min limite value
  * @retval     NULL
  */
#define LimitMaxMin(input, max, min) {      \
        if (input > max) {                  \
            input = max;                    \
        }                                   \
        else if (input < min) {             \
            input = min;                    \
        }                                   \
    }


typedef struct {
    float acc;		//定义速度增加时的最大斜率（加速度上限）
    float dec;		//定义速度减少时的最大斜率（减速度上限）
} Math_SlopeParamTypeDef;

float Math_Rad2Angle(float rad);
float Math_Angle2Rad(float ang);
float Math_Fal(float e, float alpha, float zeta);
int16_t Math_Fsg(float x, float d);
int16_t Math_Sign(float Input);
void Math_InitSlopeParam(Math_SlopeParamTypeDef* pparam, float acc, float dec);
float Math_CalcSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef* pparam);
float Math_Differential(float arr[], uint8_t order, float dt);
float Math_InvSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif
