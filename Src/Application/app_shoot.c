/*
 *  Project      : Polaris
 * 
 *  file         : app_shoot.c
 *  Description  : This file contains Shooter control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 13:16:21
 */


#include "app_shoot.h"
#include "module_shoot.h"

/**
  * @brief          Gimbal task
  * @param          NULL
  * @retval         NULL
  */
void Shoot_Task(void const * argument) {

    forever {
        Shooter_Control();
      osDelay(2);
    }
}
