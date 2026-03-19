/*
 *  Project      : Polaris
 * 
 *  file         : app_remote.h
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 13:20:47
 */


#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif


#include "periph_remote.h"
#include "periph_servo.h"
#include "alg_math.h"

typedef struct {
    uint8_t pending;		//标记遥控器是否有未处理的新指令,1为有指令需要处理,0为无或已处理
    float yaw_angle_offset;	//存储偏航角的校准偏移量
} Remote_RemoteControlTypeDef;

void Remote_Task(void const * argument);
void Remote_RemotrControlInit(void);
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr(void);
void Remote_ControlCom(void);
void Remote_MouseShooterModeSet(void);
void Remote_RemoteShooterModeSet(void);
void Remote_RemoteProcess(void);
void Remote_KeyMouseProcess(void);

#endif

#ifdef __cplusplus
}
#endif
