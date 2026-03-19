/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_softTimer.c
 *  Description  : This file contains the soft timers task
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 13:49:14
 *  LastEditTime : 2023-05-05 09:26:20
 */


#include "sys_softTimer.h"
#include "main.h"

/** 
  * @brief          Start All Timer and Set cycle 
  * @retval        
 */
void SoftTimer_StartAll() {
    osTimerStart(SoftTimerHandle, 250);
}


/** 
  * @brief         For beeper Timer Callback
  * @param         argument
  * @retval        
 */
void SoftTimerCallback(void const * argument) {
    
}

