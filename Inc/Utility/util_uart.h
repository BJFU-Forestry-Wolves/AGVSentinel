/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_uart.h
 *  Description  : This file contains the functions of USART
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-23 04:50:29
 */


#ifndef UART_UTIL_H
#define UART_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "usart.h"

void Uart_RxIdleCallback(UART_HandleTypeDef* huart);
void Uart_SendMessage(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout);
void Uart_SendMessage_IT(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size);
void Uart_SendMessage_IT_Force(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout);
void Uart_ErrorHandler(uint32_t ret);
void Uart_InitUartDMA(UART_HandleTypeDef* huart);
void Uart_ReceiveDMA(UART_HandleTypeDef* huart, uint8_t rxdata[], uint32_t size);
void Uart_ReceiveHandler(UART_HandleTypeDef *huart);
uint16_t Uart_DMACurrentDataCounter(DMA_Stream_TypeDef *dma_stream);

#endif

#ifdef __cplusplus
}
#endif
