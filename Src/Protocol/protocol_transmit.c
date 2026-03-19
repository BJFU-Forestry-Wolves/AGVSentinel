/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_transmit.c
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:41
 *  LastEditTime : 2023-05-06 08:42:44
 */


#include "protocol_common.h"
#include "protocol_transmit.h"
#include "stdlib.h"
#include "lib_buff.h"
#include "util_can.h"

#include "app_ins.h"
#include "sys_dwt.h"


static void _send_mode(uint32_t stdid, uint8_t *buff);
static void _send_imu_data(uint32_t stdid, uint8_t *buff);
static void _send_chassis_ref(uint32_t stdid, uint8_t *buff);
static void _send_leg_ref(uint32_t stdid, uint8_t *buff);

const uint32_t CMD_SEND_MODE                      = 0x205;
const uint32_t CMD_SEND_IMU_YAW                   = 0x101;
const uint32_t CMD_SEND_CHA_REF                   = 0x103;
const uint32_t CMD_SEND_LEG_REF                   = 0X105;
const uint32_t Const_BusComm_CAN_TX_EXTID           = 0x01;

CAN_TxHeaderTypeDef Protocol_CAN_TxhEADER;


Protocol_SendEntry ProtocolCmd_Send[Const_Protocol_Transmit_BUFF_SIZE] = {
    {CMD_SEND_MODE   ,          &_send_mode         },
    {CMD_SEND_IMU_YAW,          &_send_imu_data     },
    {CMD_SEND_CHA_REF,          &_send_chassis_ref  },
    {CMD_SEND_LEG_REF,          &_send_leg_ref      }
};          


uint32_t dt_tick3;
float dt_3;
static void _send_mode(uint32_t stdid, uint8_t *buff) {
    dt_3 = DWT_GetDeltaT(&dt_tick3);
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Can_InitTxHeader(&Protocol_CAN_TxhEADER, stdid, Const_BusComm_CAN_TX_EXTID, 8);
    uint8_t send_buff[8];
    send_buff[0] = (((uint8_t)buscomm->bal_mode) << 4) | ((uint8_t)buscomm->cha_mode);
    send_buff[1] = buscomm->yaw_control | (buscomm->yaw_output << 1);
    Can_SendMessage(&hcan2, &Protocol_CAN_TxhEADER, send_buff);
}


uint32_t dt_tick4;
float dt_4;
static void _send_imu_data(uint32_t stdid, uint8_t *buff) {
    dt_4 = DWT_GetDeltaT(&dt_tick4);
    INS_INSTypeDef *ins = INS_GetINSPtr();
    Can_InitTxHeader(&Protocol_CAN_TxhEADER, stdid, Const_BusComm_CAN_TX_EXTID, 8);
    uint8_t send_buff[8];
    float2buff(ins->Gyro[Z_INS], send_buff);
    float2buff(ins->YawTotalAngle, send_buff + 4);
    Can_SendMessage(&hcan2, &Protocol_CAN_TxhEADER, send_buff);
}


uint32_t dt_tick5;
float dt_5;
static void _send_chassis_ref(uint32_t stdid, uint8_t *buff) {
    dt_5 = DWT_GetDeltaT(&dt_tick5);
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Can_InitTxHeader(&Protocol_CAN_TxhEADER, stdid, Const_BusComm_CAN_TX_EXTID, 8);
    uint8_t send_buff[8];
    float2buff(buscomm->dx, send_buff);                  //把dx
    float2buff(buscomm->yaw_ref, send_buff + 4);         //yaw_ref发出去
    Can_SendMessage(&hcan2, &Protocol_CAN_TxhEADER, send_buff);
}


uint32_t dt_tick6;
float dt_6;
static void _send_leg_ref(uint32_t stdid, uint8_t *buff) {
    dt_6 = DWT_GetDeltaT(&dt_tick6);
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Can_InitTxHeader(&Protocol_CAN_TxhEADER, stdid, Const_BusComm_CAN_TX_EXTID, 8);
    uint8_t send_buff[8];
    float2buff(buscomm->leg_len, send_buff);
    Can_SendMessage(&hcan2, &Protocol_CAN_TxhEADER, send_buff);
}
