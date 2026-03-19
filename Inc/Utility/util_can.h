/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_can.h
 *  Description  : This file contains the functions of CAN
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-03-13 12:27:08
 */


#ifndef CAN_UTIL_H
#define CAN_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "can.h"

void Can_ErrorHandler(uint32_t ret);
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc);
void Can_InitFilterAndStart(CAN_HandleTypeDef* phcan);
void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* phcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan);
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]);

#endif

#ifdef __cplusplus
}
#endif
