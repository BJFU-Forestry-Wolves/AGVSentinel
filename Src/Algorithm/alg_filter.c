/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_filter.c
 *  Description  : This document contains digital filter correlation function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:06
 *  LastEditTime : 2023-03-25 20:47:18
 */


#include "alg_filter.h"
#include "alg_math.h"


/**
  * @brief      low_pass_filter_init
  * @param      param :Low pass filter param
  * @param      Filter_LowPassParamTypeDef: low pass filter param stuct
  * @retval     filtering result
  */
void Filter_LowPassInit(float param, Filter_LowPassParamTypeDef *pparam) {
    pparam->filt_para = param;
}


/**
  * @brief      low_pass_filter
  * @param      val: inital value 
  * @param      pparam : the low pass filter param sturct
  * @param      filt: low pass filter sturct
  * @retval     filtering result
  */
float Filter_LowPass(float val, Filter_LowPassParamTypeDef *pparam, Filter_LowPassTypeDef *filt) {
    if ((pparam->filt_para <= 0) || (pparam->filt_para >= 1)) return val;
    filt->filted_val = pparam->filt_para * val + (1 - pparam->filt_para) * filt->filted_last_val;
    filt->filted_last_val = filt->filted_val;
    return filt->filted_val;

}


/**
  * @brief      average_filter
  * @param      val  :inital value 
  * @param      filt :average_filter sturct
  * @retval     filtering result
  */
float Filter_Aver(float val, Filter_WindowTypeDef *filt) {
    filt->sum = 0;
    for (int i = 0; i < MAX_LENGTH - 1; i++) {
        filt->val[i] = filt->val[i + 1];
    }
    filt->val[MAX_LENGTH-1] = val;
    for (int i = 0 ; i < MAX_LENGTH ; i++) {
        filt->sum += filt->val[i];
    }
    return filt->sum / MAX_LENGTH;
}


/**
  * @brief      bessel_filter
  * @param      val  :inital value 
  * @param      filt :bessel_filter sturct
  * @retval     filtering result
  */
float Filter_Bessel(float val, Filter_Bessel_TypeDef *filt) {
    for (int i = 3; i > 0; i--) {
        filt->xbuf[i] = filt->xbuf[i-1];
        filt->ybuf[i] = filt->ybuf[i-1];
    }
    filt->xbuf[0] = val;
    filt->ybuf[0] = 0.0001507f * filt->xbuf[1] + 0.0005675f * filt->xbuf[2] + 0.0001336f * filt->xbuf[3] 
                    + 2.765f * filt->ybuf[1] - 2.552f * filt->ybuf[2] + 0.7866f * filt->ybuf[3];
    filt->filted_val = filt->ybuf[0];
    return filt->filted_val;
}
