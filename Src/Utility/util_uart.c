/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_uart.c
 *  Description  : This file contains the functions of USART
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-05-05 09:37:27
 */


#include "util_uart.h"
#include "periph_remote.h"
#include "periph_referee.h"
#include "periph_pc_comm.h"

/**
  * @brief      UART RX Callback allocation function
  * @param      huart: uart IRQHandler id
  * @retval     NULL
  */
void Uart_RxIdleCallback(UART_HandleTypeDef* huart) {
    if (huart == Const_Remote_UART_HANDLER) {
        Remote_RXCallback(huart);
    }
    if (huart == Const_PC_Comm_UART_HANDLER) {
        PC_Comm_RXCallback(huart);
    }
}


/********** VOLATILE USER CODE END **********/


/**
  * @brief      Sending information to UART (blocking mode)
  * @param      huart: UART handle
  * @param      txdata: The message to send
  * @param      size: The message length
  * @param      timeout: Timeout duration
  * @retval     NULL
  */
void Uart_SendMessage(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout) {
    /* Start the Transmission process */
    uint32_t ret = HAL_UART_Transmit(huart, txdata, size, timeout);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Uart_ErrorHandler(ret);
    }
}


/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param      dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t Uart_DMACurrentDataCounter(DMA_Stream_TypeDef *dma_stream) {
    /* Return the number of remaining data units for DMAy Streamx */
    return ((uint16_t)(dma_stream->NDTR));
}


/**
  * @brief      Sending information to UART (Non blocking mode)
  * @param      huart: UART handle
  * @param      txdata: The message to send
  * @param      size: The message length
  * @retval     NULL
  */
void Uart_SendMessage_IT(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size) {
    /* Start the Transmission process */
    uint32_t ret = HAL_UART_Transmit_IT(huart, txdata, size);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Uart_ErrorHandler(ret);
    }
}


/**
  * @brief      Sending information to UART (Non blocking mode)锟斤拷force waiting锟斤拷may cause delay
  * @param      huart: UART handle
  * @param      txdata: The message to send
  * @param      size: The message length
  * @param      timeout: Timeout duration
  * @retval     NULL
  */
void Uart_SendMessage_IT_Force(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout) {
//    /* Start the Transmission process */
//    uint32_t now = HAL_GetTick();
//    uint32_t ret;
//    do {
//        ret = HAL_UART_Transmit_IT(huart, txdata, size);
//    } while (ret != HAL_OK && HAL_GetTick() - now <= timeout);
//    if (ret != HAL_OK) {
//        /* Transmission request Error */
//        Uart_ErrorHandler(ret);
//    }
    /* Start the Transmission process */
    __HAL_UNLOCK(huart);
    uint32_t ret = HAL_UART_Transmit_IT(huart, txdata, size);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Uart_ErrorHandler(ret);
    }
}


/**
  * @brief      UART error handler
  * @param      ret: error data
  * @retval     NULL
  */
void Uart_ErrorHandler(uint32_t ret) {
    //Log_DebugPrintf("Error: UART Error!\n");
    while (1) {
        return;
    }
}


/**
  * @brief      initialization UART DMA
  * @param      huart: UART handle
  * @retval     NULL
  */
void Uart_InitUartDMA(UART_HandleTypeDef* huart) {
    /* open uart idle it */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}


/**
  * @brief      enable global uart it and do not use DMA transfer done it
  * @param      huart: uart IRQHandler id
  * @param      pData: receive buff 
  * @param      Size:  buff size
  * @retval     set success or fail
  */
void Uart_ReceiveDMA(UART_HandleTypeDef* huart, uint8_t rxdata[], uint32_t size) {
    
    uint32_t tmp1 = 0;
    tmp1 = huart->RxState;
    if (tmp1 == HAL_UART_STATE_READY) {
        if ((rxdata == NULL) || (size == 0)) {
            return;
        }
        huart->pRxBuffPtr = rxdata;
        huart->RxXferSize = size;
        huart->ErrorCode  = HAL_UART_ERROR_NONE;
        /* Enable the DMA Stream */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)rxdata, size);
        /* 
         * Enable the DMA transfer for the receiver request by setting the DMAR bit
         * in the UART CR3 register 
         */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
}


/**
  * @brief      UART RX callback receiver function
  * @param      huart: Point to uart handle
  * @retval     NULL
  */
void Uart_ReceiveHandler(UART_HandleTypeDef *huart) {
    // clear idle it flag after uart receive a frame data
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) {
        /* clear idle it flag avoid idle interrupt all the time */
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        /* handle received data in idle interrupt */
        Uart_RxIdleCallback(huart);
    }
}
