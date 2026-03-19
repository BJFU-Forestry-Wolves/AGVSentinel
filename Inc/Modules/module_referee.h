/*
 *  Project      : Polaris
 * 
 *  file         : module_referee.c
 *  Description  : User defined UI interface of referee system
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 11:00:18
 */


#ifndef CHA_REFEREE_CTRL_H
#define CHA_REFEREE_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define REFEREE_TASK_PERIOD 200

typedef struct {
    uint8_t width_mode, width_mode_last;        // 1 for gyro mode, 0 for normal mode
    uint8_t aim_mode, aim_mode_last;            // 0~2 correspond to 15,18,30 m/s bullet speed
    uint8_t auto_aim_mode, auto_aim_mode_last;
    uint8_t cha_mode, cha_mode_last;
    uint8_t cap_state;                          // cap percent, 0 ~ 100
    float pitch_angle;
} Referee_DrawDataTypeDef;


void Referee_Task(void const * argument);

void Referee_SetWidthMode(uint8_t mode);
void Referee_SetAimMode(uint8_t mode);
void Referee_SetCapState(uint8_t state);
void Referee_SetPitchAngle(float angle);
void Referee_SetMode(uint8_t auto_aim_mode, uint8_t cha_mode);

void Referee_SetupAimLine(void);
void Referee_UpdateAimLine(void);
void Referee_SetupCrosshair(void);
void Referee_UpdateCrosshair(void);
void Referee_SetupWidthMark(void);
void Referee_UpdateWidthMark(void);
void Referee_SetupCapState(void);
void Referee_UpdateCapState(void);
void Referee_SetupPitchMeter(void);
void Referee_UpdatePitchMeter(void);
void Referee_SetupModeDisplay(void);
void Referee_UpdateModeDisplay(void);
void Referee_SetupErrorDisplay(void);
void Referee_UpdateErrorDisplay(void);
void Referee_SetupAllString(void);
void Referee_Setup(void);
void Referee_Update(void);

#endif

#ifdef __cplusplus
}
#endif
