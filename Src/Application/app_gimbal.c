/*
 *  Project      : Polaris
 * 
 *  file         : app_gimbal.c
 *  Description  : This file contains Gimbal Pitch control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 13:13:54
 */


#include "app_gimbal.h"
#include "module_gimbal.h"
#include "app_autoaim.h"
#include "module_shoot.h"
#include "periph_DMmotor.h"
/**
  * @brief          Gimbal task
  * @param          NULL
  * @retval         NULL
  */
void Gimbal_Task(void const * argument) {       //最高优先级   哨兵的任务
        MyUART_Init();
    for(;;) {
			  Tidy_send_vision(&visionDataSend);     //将陀螺仪各个数据存入裁判系统
			  SendVisionData(&visionDataSend);       //将数据发给视觉

		dm_motor_detect(&motor[Motor1]);
		Shooter_UpdataControlData();
        GimbalPitch_Control();
		GimbalYaw_Control();
		Shooter_FeederControl();
        GimbalYaw_Output();
        dm_motor_ctrl_send(&hcan2, &motor[Motor1]);
      osDelay(1);
    }
}
