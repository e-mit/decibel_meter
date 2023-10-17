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

#ifdef DEBUG_AND_TESTS
#ifndef HAL_UART_MODULE_ENABLED
#error("Enabling debug and tests means that UART must be enabled also.")
#endif
#endif


int main(void) {
	if (HAL_Init() != HAL_OK) {
		errorHandler(__func__, __LINE__, __FILE__);
	}
	if (!SystemClock_Config()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	GPIO_Init();

#ifdef HAL_UART_MODULE_ENABLED
	if (!UART_Init()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}
#endif


#ifdef DEBUG_AND_TESTS
	getStartupReason();
#endif

	while (true) {
		PRINT("Hello.\n");
		HAL_Delay(1000);
	}

	//////////////////////////////////////////////////////////

	HAL_Delay(20); // allow voltages to settle prior to I2C transactions


	// initialise microphone (no I2S clock or DMA interrupts enabled yet)
	if (!sound_init()) {
		PRINT("sound_init() failed.\n");
	}

	/////////////////////////////////////

	if (!applyDefaultSettings()) {
		PRINT("applyDefaultSettings() failed.\n");
	}

	/////////////////////////////////////

	while (true) {
		// update stable status so it can be read at any time (not just after a data readout)
		soundData.stable = micHasStabilized() ? 1 : 0;

		uint32_t staged_IAQ_readout_wait = staged_IAQ_readout();

		#ifdef ENABLE_SLEEP_WAITING
			// sleep indefinitely until an interrupt occurs (NB: systick will wake it)
			// do not sleep if either of the staged readouts has just returned a request for zero wait
			if (staged_IAQ_readout_wait != 0) {
				HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			}
		#endif
	}
	return 0;
}

