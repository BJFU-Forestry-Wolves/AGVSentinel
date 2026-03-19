/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_init.h
 *  Description  : All initialization threads
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:43:10
 *  LastEditTime : 2023-05-05 10:54:07
 */


#ifndef APP_INIT_H
#define APP_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


void Init_InitAll(void);
void Init_Task(void const * argument);

#endif

#ifdef __cplusplus
}

#endif
