/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_communicate.c
 *  Description  : Communication Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:44:07
 *  LastEditTime : 2023-05-06 08:35:13
 */

//与视觉的连接   最高优先级ospriorityrealtime
#include "app_communicate.h"
#include "protocol_common.h"
#include "sys_dwt.h"

void Comm_Task(void const * argument) {
    forever {
        Protocol_SendProtocolData();
      osDelay(1);
    }
}
