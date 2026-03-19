/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_communicate.h
 *  Description  : Communication Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:44:21
 *  LastEditTime : 2023-02-08 00:10:27
 */


#ifndef APP_COMMUNICATE_H
#define APP_COMMUNICATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"



void Comm_Task(void const * argument);


#endif

#ifdef __cplusplus
}

#endif
