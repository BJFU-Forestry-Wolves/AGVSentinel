#include "app_chassis.h"
#include "module_chassis.h"


/**
  * @brief          Chassis task
  * @param          NULL
  * @retval         NULL
  */
void Chassis_Task(void const * argument) {

    for(;;) {
			Chassis_Control();
      Chassis_Output();//CAN下发四个电机输出
      osDelay(2);
    }
}
