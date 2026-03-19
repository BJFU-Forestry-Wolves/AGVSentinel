/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_receive.h
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:43
 *  LastEditTime : 2023-05-05 10:57:00
 */


#ifndef PROTOCOL_RECEIVE_H
#define PROTOCOL_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Receive_BUFF_SIZE 2

extern const uint32_t CMD_SET_MODE;        
extern const uint32_t CMD_SET_GIMBAL_DATA; 
extern const uint32_t CMD_SET_IMU_YAW;    
extern const uint32_t CMD_SET_CHA_REF;     

typedef struct {
    uint32_t cmd_id;                    
    void (*bus_func)(uint8_t *buff);
} Protocol_ReceiveEntry;

extern Protocol_ReceiveEntry ProtocolCmd_Receive[Const_Protocol_Receive_BUFF_SIZE];


#endif

#ifdef __cplusplus
}
#endif
