#include "main.h"
#include "mainpp.h"
#include "stm32h7xx_hal.h"

#include "ROS_mainpp.h"
#include "DebugMode.h"
#include "Omni.h"

// For LED blink
extern TIM_HandleTypeDef htim8;

void main_function() {
	// Init
	ROS::setup();
	omni.Init();

	// For LED blink
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

#ifdef DEBUGGER_MODE
	DebugMode.DebuggingMode();
	return;
#endif

	while (true) {
	}
}
