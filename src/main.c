#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdlib.h>
#include "stm32g0xx_hal.h"
#include "hardware_profile.h"
#include "print_functions.h"
#include "interrupts.h"
#include "arm_math.h"
#include "sound_LUTs.h"
#include "sound_test_data.h"
#include "arm_const_structs.h"
#include "sound_measurement.h"
#include "math.h"
#include "sensor_constants.h"
#include "UART.h"
#include "utilities.h"
#include "MS_functions.h"

SoundData_t soundData = {0};

int main(void) {
	// Initialise the system
	if (HAL_Init() != HAL_OK) {
		errorHandler(__func__, __LINE__, __FILE__);
	}
	if (!SystemClock_Config()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}
	GPIO_Init();
	if (!UART_Init()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}
	#ifdef DEBUG_PRINT
		const char * startupReason = getStartupReason();
		printSerial("Reset flag(s): %s\n", startupReason);
	#endif

	if (!sound_init()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	if (!enableMicrophone(true)) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	while (true) {

		getSoundDataStruct(&soundData, true, true);
		if (true) {
			printAllData(&soundData);
			clearMaxAmpFollower(); // move this into function
		}

		// Sleep until next interrupt
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}
	return 0;
}

