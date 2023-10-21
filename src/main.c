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

bool simplified_readout(void);
SoundData_t soundData_g = {0};

int main(void) {
	if (HAL_Init() != HAL_OK) {
		errorHandler(__func__, __LINE__, __FILE__);
	}
	if (!SystemClock_Config()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	if (!UART_Init()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	if (!sound_init()) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	if (!enableMicrophone(true)) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	while (true) {

		if (simplified_readout()) {
			clearMaximumAmplitude(); // Call this at any time
			printSerial("%u.%u  %u.%02u  %u\n", soundData_g.SPL_dBA_int,
					    soundData_g.SPL_dBA_fr_1dp, soundData_g.peak_amp_mPa_int,
						soundData_g.peak_amp_mPa_fr_2dp, soundData_g.stable);
		}

	}
	return 0;
}

// Return bool: new data available
bool simplified_readout(void) {
	static uint32_t stage = 0;

	if (stage == 0) {
		enableSPLcalculation(true); // start a new acquisition/calculation
		stage++;
		return false;
	}
	else if (stage == 1) {
		if (!isSPLcalcComplete()) {
			return false;
		}
		// Acquisition/calculation complete: get data
		getSoundDataStruct(&soundData_g, true, true);
		enableSPLcalculation(false); // reset the calculation, ready for restart
		stage = 0;
		return true;
	}

	errorHandler(__func__, __LINE__, __FILE__);
	stage = 0;
	return false;
}

