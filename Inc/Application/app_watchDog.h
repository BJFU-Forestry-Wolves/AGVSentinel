/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_watchDog.h
 *  Description  : Watch Dog Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:45:32
 *  LastEditTime : 2023-02-08 00:05:42
 */


#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


void WatchDog_Task(void const * argument);
void WatchDog_Feed(void);
uint8_t WatchDoge_CheckDogBowl(void);

#endif

#ifdef __cplusplus
}

#endif
