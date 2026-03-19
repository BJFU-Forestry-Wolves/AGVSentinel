#ifndef APP_REFEREE_H
#define APP_REFEREE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"

void Referee_Task(void const * argument);
void Referee_ResetSetup(void);
#endif

#ifdef __cplusplus
}
#endif

