/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_pid.c
 *  Description  : This file contains PID algorithm function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:06
 *  LastEditTime : 2023-01-23 04:55:45
 */


#include "alg_pid.h"
#include "stddef.h"
#include "string.h"


/**
  * @brief      Initialize PID control parameters
  * @param      param: The pointer points to PID control parameters
  * @param      kp: P factor
  * @param      ki: I factor
  * @param      kd: D factor
  * @param      sum_max: Integral limiting
  * @param      output_max: Output limiting
  * @param      kd_fil_frq : PID Kd out filter
  * @param      delta_fil_frq: Delta PID ref filter
  * @param      kf_1: Feedforward of first order param
  * @param      kf_2: Feedforward of second order param
  * @param      kf1_fil_frq: Feedforward first order filter
  * @param      kf2_fil_frq: Feedforward second order filter
  * @param      period: PID calculate period
  * @param      pid_mode: PID calculate mode
  * @retval     NULL
  */
void PID_InitPIDParam(PID_PIDParamTypeDef* pparam, float kp, float ki, float kd, float sum_max, float output_max, 
                      float kd_fil_param, float delta_fil_param, float kf_1, float kf_2, float kf1_fil_param, 
                      float kf2_fil_param,PID_ModeEnum pid_mode) {

    pparam->pid_mode = pid_mode;
    pparam->kp = kp;
    pparam->ki = ki;
    pparam->kd = kd;

    pparam->sum_max    = sum_max;
    pparam->output_max = output_max;

    Filter_LowPassInit(kd_fil_param, &pparam->d_fil_param);
    Filter_LowPassInit(delta_fil_param, &pparam->delta_fil_param);

    pparam->kf_1 = kf_1;
    pparam->kf_2 = kf_2;

    Filter_LowPassInit(kf1_fil_param, &pparam->kf1_fil_param);
    Filter_LowPassInit(kf2_fil_param, &pparam->kf2_fil_param);
    
}


/**
  * @brief      Get PID control value
  * @param      pid: The pointer points to the PID controller
  * @retval     Set value of PID control quantity
  */
float PID_GetPIDRef(PID_PIDTypeDef* pid) {
    return pid->ref;
}


/**
  * @brief      Setting PID control value
  * @param      pid: The pointer points to the PID controller
  * @param      ref: set value
  * @retval     NULL
  */
void PID_SetPIDRef(PID_PIDTypeDef* pid, float ref) {
    pid->ref = ref;//目标值
}


/**
  * @brief      Increase the set value of PID control value (the increment can be negative)
  * @param      pid: The pointer points to the PID controller
  * @param      inc: Increment value
  * @retval     NULL
  */
void PID_AddPIDRef(PID_PIDTypeDef* pid, float inc) {
    pid->ref += inc;
}


/**
  * @brief      Obtaining feedback value of PID control quantity
  * @param      pid: The pointer points to the PID controller
  * @retval     Feedback value of PID control quantity
  */
float PID_GetPIDFdb(PID_PIDTypeDef* pid) {
    return pid->fdb;
}


/**
  * @brief      Setting feedback value of PID control quantity
  * @param      pid: The pointer points to the PID controller
  * @param      fdb: Feedback value
  * @retval     NULL
  */
void PID_SetPIDFdb(PID_PIDTypeDef* pid, float fdb) {
    pid->fdb = fdb;//实际值
}


/**
  * @brief      Obtain PID control output value
  * @param      pid: The pointer points to the PID controller
  * @retval     Output value of PID control quantity
  */
float PID_GetPIDOutput(PID_PIDTypeDef* pid) {
    return pid->output;
}


/**
  * @brief      Initialize PID controller
  * @param      pid: The pointer points to the PID controller
  * @retval     NULL
  */
void PID_ClearPID(PID_PIDTypeDef* pid) {
    pid->ref        = 0;
    pid->fdb        = 0;
    pid->err[0]     = 0;
    pid->err[1]     = 0;
    pid->err[2]     = 0;
    pid->err_lim    = 0;
    pid->err_fdf[0] = 0;
    pid->err_fdf[1] = 0;
    pid->err_fdf[2] = 0;
    pid->out_fdf    = 0;
    pid->sum        = 0;
    pid->output     = 0;


    pid->err_watch  = 0;
}


/**
  * @brief      Calculation of PID control quantity
  * @param      pid: The pointer points to the PID controller
  * @param      para: The pointer points to PID control parameters
  * @retval     NULL
  */
void PID_CalcPID(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* pparam) {

    //位置式
    if (pparam->pid_mode == PID_POSITION) {
        float dError, Error, ref_dError, ref_ddError;

        // Calculate the differential value
        Error = pid->ref - pid->fdb;
        pid->err[2] = pid->err[1];
        pid->err[1] = pid->err[0];
        pid->err[0] = Error;

        dError = Math_Differential(pid->err, 1, 1);


        pid->err_fdf[2] = pid->err_fdf[1];
        pid->err_fdf[1] = pid->err_fdf[0];
        pid->err_fdf[0] = pid->ref;

        ref_dError = Math_Differential(pid->err_fdf, 1, 1);
        ref_ddError = Math_Differential(pid->err_fdf, 2, 1);

        pid->err_watch = Error;
        // Calculate the integral and integra anti-windup 
        if (pparam->kp == 0)
            pid->sum = pid->sum + Error;
        else
            pid->sum = pid->sum + Error + pid->err_lim / pparam->kp;

        // Integral limiting
        LimitMax(pid->sum, pparam->sum_max);

        // Calculation results kf1_filter
        pid->out_fdf = Filter_LowPass((pparam->kf_1 * ref_dError), &pparam->kf1_fil_param, &pid->kf1_fil) + Filter_LowPass((pparam->kf_2 * ref_ddError), &pparam->kf2_fil_param, &pid->kf2_fil);
        pid->output  = pparam->kp * Error + pparam->ki * pid->sum + pparam->kd * Filter_LowPass(dError, &pparam->d_fil_param, &pid->d_fil) + pid->out_fdf;        
    }
    
//增量式
    else if (pparam->pid_mode == PID_DELTA) {
        float dError, ddError, Error, ref_dError, ref_ddError;

        // Calculate the difference
        Error = pid->ref - pid->fdb;
        pid->err[2] = pid->err[1];
        pid->err[1] = pid->err[0];
        pid->err[0] = Error;

        dError  = Filter_LowPass(Math_Differential(pid->err, 1, 1), &pparam->delta_fil_param, &pid->delta_fil);
        ddError = Math_Differential(pid->err, 2, 1);


        pid->err_fdf[2] = pid->err_fdf[1];
        pid->err_fdf[1] = pid->err_fdf[0];
        pid->err_fdf[0] = pid->ref;
        
        ref_dError = Math_Differential(pid->err_fdf, 1, 1);
        ref_ddError = Math_Differential(pid->err_fdf, 2, 1);
    
        pid->err_watch = Error;

        // Calculate the integral and integral anti-windup 
        if (pparam->kp == 0)
            pid->sum = Error;
        else
            pid->sum = Error + pid->err_lim / pparam->kp;

        // Integral limiting
        LimitMax(pid->sum, pparam->sum_max);
    
        // Calculation results kf1_filter
        pid->out_fdf =  Filter_LowPass((pparam->kf_1 * ref_dError), &pparam->kf1_fil_param, &pid->kf1_fil) + Filter_LowPass((pparam->kf_2 * ref_ddError), &pparam->kf2_fil_param, &pid->kf2_fil);
        pid->output += (pparam->kp * dError + pparam->ki * pid->sum + pparam->kd * Filter_LowPass(ddError, &pparam->d_fil_param, &pid->d_fil));
        pid->output += pid->out_fdf;
    }    

        // Output limiting
    float temp_limit = pid->output;
    LimitMax(pid->output, pparam->output_max);
    pid->err_lim = pid->output - temp_limit;   
}

