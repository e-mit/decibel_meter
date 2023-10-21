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

#ifdef DEBUG_AND_TESTS
#ifndef HAL_UART_MODULE_ENABLED
#error("Enabling debug and tests means that UART must be enabled also.")
#endif
#endif

uint32_t simplified_readout(void);
volatile bool soundDataReady = false;
SoundData_t soundData = {0};

int main(void) {
	if (HAL_Init() != HAL_OK) {
		Error_Handler();
	}
	if (!SystemClock_Config()) {
		Error_Handler();
	}

	GPIO_Init();

	if (!UART_Init()) {
		Error_Handler();
	}

	if (!sound_init()) {
		PRINT("sound_init() failed.\n");
	}

	/////////////////////////////////////

	if (!enableMic(true)) {
		return false;
	}

	enable_I2S_DMA_interrupts(true);
	clearMaxAmpFollower();

	while (true) {

		simplified_readout();
		if (soundDataReady) {
			printSerial("%u.%u\n", soundData.SPL_dBA_int, soundData.SPL_dBA_fr_1dp);
			soundDataReady = false;
		}

	}
	return 0;
}


uint32_t simplified_readout(void) {
	static uint32_t stage = 0;

	if (stage == 0) {
		if (!soundDataReady) {
			enableSPLcalculation(true); // this also resets the sound filter (if used) and clears SPL semaphore
			stage++;
		}
		return 0;
	}
	else if (stage == 1) {
		if (!is_SPL_calc_complete()) {
			return 0;
		}
		uint32_t maxAmp_DN; // unused here
		getSoundDataStruct(&soundData, true, true, &maxAmp_DN);
		clearMaxAmpFollower();
		enableSPLcalculation(false);
		soundDataReady = true;
		stage = 0;
		return 0;
	}

	// reaching here is an error: reset the cycles:
	PRINT("Cycle error: setting stage = 0.\n");
	stage = 0; // restart as if just entering cycle mode
	return 0;
}

