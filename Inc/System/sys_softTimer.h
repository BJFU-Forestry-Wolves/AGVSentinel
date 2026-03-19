/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_softTimer.h
 *  Description  : This file contains the soft timers task
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 13:49:27
 *  LastEditTime : 2023-05-05 09:25:45
 */


#ifndef SYS_SOFTTIMER_H
#define SYS_SOFTTIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"

void SoftTimer_StartAll(void);
void SoftTimerCallback(void const * argument);


#endif

#ifdef __cplusplus
}
#endif
