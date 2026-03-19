/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_crc.h
 *  Description  : This file contains crc check function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-29 03:36:12
 */


#ifndef CRC_ALG_H
#define CRC_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


extern uint8_t CRC8;
extern uint16_t CRC16;
extern uint16_t CRC16_INIT;
extern uint8_t CRC8_INIT;

typedef enum {
        NOT_MATCH = 0,
        MATCH     = 1
} CRC_MatchEnum;


uint8_t CRC_GetCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint8_t CRC_VerifyCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);
void CRC_AppendCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t CRC_GetCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
unsigned int CRC_VerifyCRC16CheckSum(unsigned char *pchMessage, unsigned int dwLength);
void CRC_AppendCRC16CheckSum(unsigned char *pchMessage, unsigned int dwLength);
CRC_MatchEnum CRC_VerifyIMU_HI229(uint8_t* buff);
void crc16_verify(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);

#ifdef __cplusplus
}
#endif

#endif

