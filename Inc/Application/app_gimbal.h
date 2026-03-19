/*
 *  Project      : Polaris
 * 
 *  file         : app_gimbal.h
 *  Description  : This file contains Gimbal control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 13:14:08
 */

#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "cmsis_os.h"
#include "FreeRTOS.h"

void Gimbal_Task(void const * argument);


#endif

#ifdef __cplusplus
}
#endif
