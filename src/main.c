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

void simplified_readout(void);
volatile bool soundDataReady = false;
SoundData_t soundData = {0};

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

		simplified_readout();
		if (soundDataReady) {
			printSerial("%u.%u  %u.%02u  %u\n", soundData.SPL_dBA_int, soundData.SPL_dBA_fr_1dp,
						soundData.peak_amp_mPa_int, soundData.peak_amp_mPa_fr_2dp, soundData.stable);
			soundDataReady = false;
		}

	}
	return 0;
}


void simplified_readout(void) {
	static uint32_t stage = 0;

	if (stage == 0) {
		if (!soundDataReady) {
			enableSPLcalculation(true); // this also resets the sound filter (if used) and clears SPL semaphore
			stage++;
		}
		return;
	}
	else if (stage == 1) {
		if (!is_SPL_calc_complete()) {
			return;
		}
		getSoundDataStruct(&soundData, true, true);
		clearMaximumAmplitude();
		enableSPLcalculation(false);
		soundDataReady = true;
		stage = 0;
		return;
	}

	errorHandler(__func__, __LINE__, __FILE__);
	stage = 0;
	return;
}

