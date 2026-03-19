/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_pwm.c
 *  Description  : This file contains the functions of PWM
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-05-07 08:58:33
 */


#include "util_pwm.h"


/**
  * @brief      Initialize PWM
  * @param      pwm: The pointer points to the PWM object to initialize
  * @param      htim:Timer object for PWM
  * @param      ch:  The PWM channel
  * @retval     NULL
  */
void PWM_InitPWM(PWM_PWMHandleTypeDef* pwm, TIM_HandleTypeDef* htim, uint32_t ch) {
    pwm->htim   = htim;
    pwm->ch     = ch;
    pwm->state  = PWM_OFF;
    pwm->duty   = 0.0;
    pwm->clk    = 84000000;     // APB1
    if (pwm->htim == &htim1 || pwm->htim == &htim8) pwm->clk = 168000000;   // APB2: TIM1, 8~11
    pwm->period = 9999;
    
    pwm->conf.OCMode        = TIM_OCMODE_PWM1;
    pwm->conf.OCPolarity    = TIM_OCPOLARITY_HIGH;
    pwm->conf.OCFastMode    = TIM_OCFAST_DISABLE;
    
    PWM_SetPWMFreq(pwm, 1000);
}


/**
  * @brief      Start PWM output
  * @param      pwm: The pointer points to the PWM object to initialize 
  * @retval     NULL
  */
void PWM_StartPWM(PWM_PWMHandleTypeDef* pwm) {
    if (pwm->state == PWM_OFF) {
        HAL_TIM_PWM_Start(pwm->htim, pwm->ch); 
        pwm->state = PWM_ON;
    }
}


/**
  * @brief      Stop PWM output
  * @param      pwm: The pointer points to the PWM object to initialize 
  * @retval     NULL
  */
void PWM_StopPWM(PWM_PWMHandleTypeDef* pwm) {
    if (pwm->state == PWM_ON) {
        HAL_TIM_PWM_Stop(pwm->htim, pwm->ch);
        pwm->state = PWM_OFF;
    }
}


/**
  * @brief      Setting PWM duty
  * @param      pwm: The pointer points to the PWM object to initialize 
  * @param      duty: PWM duty 
  * @retval     NULL
  */
void PWM_SetPWMDuty(PWM_PWMHandleTypeDef* pwm, float duty) {
    if (duty == pwm->duty) return;
    PWM_PWMStateEnum last_state = pwm->state;
    PWM_StopPWM(pwm);
    pwm->duty = duty;
    
    pwm->conf.Pulse = pwm->duty * (pwm->htim->Init.Period + 1);
    HAL_TIM_PWM_ConfigChannel(pwm->htim, &(pwm->conf), pwm->ch);
    
    if (last_state == PWM_ON) {
        PWM_StartPWM(pwm);
    }
}


/**
  * @brief      Setting PWM frequency
  * @param      pwm: The pointer points to the PWM object to initialize 
  * @param      freq: PWM frequency 
  * @retval     NULL
  */
void PWM_SetPWMFreq(PWM_PWMHandleTypeDef* pwm, uint32_t freq) {
    if (freq == pwm->freq) return;
    PWM_PWMStateEnum last_state = pwm->state;
    if (freq == 0) {
        PWM_SetPWMDuty(pwm, 0);
        return;
    }

    PWM_StopPWM(pwm);
    pwm->freq = freq;

    pwm->htim->Init.Period = pwm->period;
    pwm->htim->Init.Prescaler = pwm->clk / (pwm->htim->Init.Period + 1) / freq - 1;
    HAL_TIM_PWM_Init(pwm->htim);
    
    PWM_SetPWMDuty(pwm, pwm->duty);
    
    if (last_state == PWM_ON) {
        PWM_StartPWM(pwm);
    }
}
