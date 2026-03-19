/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_flash.h
 *  Description  : This file contains the functions of Flash
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-23 04:49:05
 */


#ifndef FLASH_UTIL_H
#define FLASH_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base address of Sector 7, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base address of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base address of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */

#define FLASH_END_ADDR ((uint32_t)0x08100000)       /* Base address of Sector 23, 128 Kbytes */

#define FLASH_READDATA(add, x) Flash_ReadData((add), (&(x)), sizeof(x))
#define FLASH_WRITEDATA(add, x) Flash_WriteSingleAddress((add), (&(x)), sizeof(x))

void Flash_EraseAddress(uint32_t address, uint16_t page);
uint8_t Flash_WriteSingleAddress(uint32_t start_address, uint32_t *buff, uint32_t len);
uint8_t Flash_WriteMuliAddress(uint32_t start_address, uint32_t end_address, uint32_t *buff, uint32_t len);
void Flash_ReadData(uint32_t address, uint32_t *buff, uint32_t len);
uint32_t Flash_GetSector(uint32_t address);
uint32_t Flash_GetNextSector(uint32_t address);

#endif

#ifdef __cplusplus
}
#endif
