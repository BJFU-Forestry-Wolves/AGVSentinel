#include "app_referee.h"
#include "module_referee.h"

static uint8_t referee_reset_flag = 0;

void Referee_Task(void const * argument) {

	
	forever{
		if (referee_reset_flag) {
			Referee_Setup();
			referee_reset_flag = 0;
			HAL_Delay(100);
		}
		Referee_Update();
		osDelay(200);
	}
}

void Referee_ResetSetup() {
	referee_reset_flag = 1;
}
