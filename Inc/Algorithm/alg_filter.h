/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_filter.h
 *  Description  : This file contains digital filter correlation function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-03-25 20:42:08
 */


#ifndef FILTER_ALG_H
#define FILTER_ALG_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

#define MAX_LENGTH 10

typedef struct {
    float filt_para;
} Filter_LowPassParamTypeDef;

typedef struct {
    float filted_val;
    float filted_last_val;
} Filter_LowPassTypeDef;

typedef struct {
    float val[MAX_LENGTH];
    float sum;
} Filter_WindowTypeDef;

typedef struct {
	double ybuf[4];
	double xbuf[4];
	float filted_val;
} Filter_Bessel_TypeDef;

void Filter_LowPassInit(float param, Filter_LowPassParamTypeDef *pparam);
float Filter_LowPass(float val, Filter_LowPassParamTypeDef *fparam, Filter_LowPassTypeDef *filt);
float Filter_Aver(float val, Filter_WindowTypeDef *filt);
float Filter_Bessel(float val, Filter_Bessel_TypeDef *filt);

#ifdef __cplusplus
}
#endif

#endif
