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

	// Enable the microphone and allow it to stabilize
	if (!enableMic(true)) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	clearAllData();
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

