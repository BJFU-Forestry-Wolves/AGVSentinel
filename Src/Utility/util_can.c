/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_can.c
 *  Description  : This file contains the functions of CAN
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-05-06 19:51:17
 */


#include "util_can.h"
#include "periph_motor.h"
#include "periph_DMmotor.h"
#include "protocol_common.h"

CAN_RxHeaderTypeDef Can_RxHeader;
#define Const_Can_RX_BUFF_LEN  200
uint8_t Can_RxData[Const_Can_RX_BUFF_LEN];


/**
 * @brief        : CAN Error handle handling
 * @param         [uint32_t] ret
 * @return        [type]
 */
void Can_ErrorHandler(uint32_t ret) {
    //Log_DebugPrintf("Error: CAN Error!\n");
    while (1) {
        return;
    }
}


/**
 * @brief        : Initialize can transmitter
 * @param         [CAN_TxHeaderTypeDef] *pheader
 * @param         [uint32_t] stdid
 * @param         [uint32_t] extid
 * @param         [uint32_t] dlc
 * @return        [type]
 */
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc) {
    pheader->StdId = stdid;
    pheader->ExtId = extid;
    pheader->RTR = CAN_RTR_DATA;
    pheader->IDE = CAN_ID_STD;
    pheader->DLC = dlc;
    pheader->TransmitGlobalTime = DISABLE;
}


/**
 * @brief        : Initialize can filter and enable CAN Bus Transceiver
 * @param         [CAN_HandleTypeDef*] phcan
 * @return        [type]
 */
void Can_InitFilterAndStart(CAN_HandleTypeDef* phcan) {
    CAN_FilterTypeDef sFilterConfig;

    if (phcan == &hcan1)
        sFilterConfig.FilterBank = 0;
    else
        sFilterConfig.FilterBank = 14;
    
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    uint32_t ret = HAL_CAN_ConfigFilter(phcan, &sFilterConfig);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_Start(phcan);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
}


/**
 * @brief        : Sending information to can bus
 * @param         [CAN_HandleTypeDef*] phcan
 * @param         [CAN_TxHeaderTypeDef*] pheader
 * @param         [uint8_t] txdata
 * @return        [type]
 */
void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]) {
    uint32_t mailbox;
    /* Start the Transmission process */
    uint32_t ret = HAL_CAN_AddTxMessage(phcan, pheader, txdata, &mailbox);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Can_ErrorHandler(ret);
    }
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan) {
		/* Get RX message */
    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {
        /* Reception Error */
        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);
}


/**
 * @brief        : HAL_CAN_ Rx Fifo0 Message Pending Call back
 * @param         [CAN_HandleTypeDef] *phcan
 * @return        [type]
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan) {
		/* Get RX message */
    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {
        /* Reception Error */
        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);		
}


/**
 * @brief        : Can bus data receiving callback function that updates the motor status according to the received information
 * @param         [CAN_HandleTypeDef*] phcan
 * @param         [CAN_RxHeaderTypeDef*] rxheader
 * @param         [uint8_t] rxdata
 * @return        [type]
 */
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]) {
    if (phcan == &hcan1) {
        Motor_EncoderDecodeCallback(phcan, rxheader -> StdId, rxdata, rxheader -> DLC);
    }
    if (phcan == &hcan2) {
 if( rxheader -> StdId==0x00)
	  {
		dm_motor_fbdata(&motor[Motor1], rxdata); 
 
	  }else  
        {
			Motor_EncoderDecodeCallback(phcan, rxheader -> StdId, rxdata, rxheader -> DLC);
		} 
    }
}
