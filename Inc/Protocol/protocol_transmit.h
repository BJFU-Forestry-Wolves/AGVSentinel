/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_transmit.h
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:52
 *  LastEditTime : 2023-05-05 13:35:37
 */


#ifndef PROTOCOL_TRANSMIT_H
#define PROTOCOL_TRANSMIT_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Transmit_BUFF_SIZE 4

typedef struct {
    uint32_t stdid;
    void (*bus_func)(uint32_t stdid, uint8_t *buff);
} Protocol_SendEntry;


extern Protocol_SendEntry ProtocolCmd_Send[Const_Protocol_Transmit_BUFF_SIZE];



#endif

#ifdef __cplusplus
}
#endif
