/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : lib_dwt.h
 *  Description  : Data Watch point and Trace
 *  LastEditors  : Polaris
 *  Date         : 2023-02-10 13:25:31
 *  LastEditTime : 2023-02-10 13:41:16
 */


#ifndef DWT_LIB_H
#define DWT_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_TimeTypeDef;

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);
static void DWT_CNT_Update(void);

#endif

#ifdef __cplusplus
}

#endif
