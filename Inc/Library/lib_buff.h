/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_buff.h
 *  Description  : This file contains auxiliary functions related to byte buffer operation
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-02-01 03:00:55
 */


#ifndef BUFF_LIB_H
#define BUFF_LIB_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "string.h"


float buff2float(uint8_t* buff);
void float2buff(float f, uint8_t* buff);
int16_t buff2i16(uint8_t* buff);
uint16_t buff2ui16(uint8_t* buff);
void ui162buff(uint16_t u, uint8_t* buff);
uint32_t buff2ui32(uint8_t* buff);
void ui322buff(uint32_t u, uint8_t* buff);

#ifdef __cplusplus
}
#endif

#endif
