/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_delay.c
 *  Description  : This file contains the functions of delay
 *  LastEditors  : Polaris
 *  Date         : 2022-04-29 12:36:14
 *  LastEditTime : 2023-02-09 12:16:30
 */


#include "lib_delay.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;


/**
 * @brief        : Initial delay param
 * @param         [type]
 * @return        [type]
 */
void Delay_Init(void) {
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;
}


/**
 * @brief        : For ns delay 
 * @param         [uint16_t] nus
 * @return        [type]
 */
void Delay_us(uint16_t nus) {
	
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            }
            else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break;
            }
        }
    } 	
}


/**
 * @brief        : For ms delay
 * @param         [uint16_t] nms
 * @return        [type]
 */
void Delay_ms(uint16_t nms) {
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            }
            else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break;
            }
        }
    }    
}
