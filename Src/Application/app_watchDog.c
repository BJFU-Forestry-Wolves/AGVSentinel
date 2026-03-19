/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_watchDog.c
 *  Description  : Watch Dog Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:45:15
 *  LastEditTime : 2023-04-07 15:27:51
 */


#include "app_watchDog.h"
#include "periph_bmi088.h"
#include "periph_motor.h"
#include "periph_remote.h"
#include "protocol_common.h"

uint16_t Dog_Bowl = 0;

void WatchDog_Task(void const * argument) {
    WatchDog_Feed();
    osDelay(50);

    forever {
        WatchDog_Feed();
      osDelay(50);
    }
}


void WatchDog_Feed() {
    if (BMI088_IsBMI088Offline() == BMI088_STATE_LOST)
        Dog_Bowl |= 1 << 0;
    if (Motor_IsAnyMotorOffline()) 
        Dog_Bowl |= 1 << 1;
    if (Remote_IsRemoteOffline() == Remote_STATE_LOST)
        Dog_Bowl |= 1 << 2;
    if (Protocol_IsProtocolOffline() == Protocol_STATE_LOST) 
        Dog_Bowl |= 1 << 3;
}

uint8_t WatchDoge_CheckDogBowl() {
    return Dog_Bowl == 0 ? 0 : 1;
}
