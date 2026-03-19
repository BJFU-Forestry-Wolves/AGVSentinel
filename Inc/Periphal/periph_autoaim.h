#ifndef PERIPH_AUTOAIM_H
#define PERIPH_AUTOAIM_H
#include "struct_typedef.h"
#include "app_autoaim.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"


extern void usart1_tx_dma_init(void);
extern void usart1_rx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
extern void usart1_rx_dma_enable(uint8_t *data, uint16_t len);

#endif
