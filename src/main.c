// Demo program for decibel sound meter project.
// Define TESTS to run system tests, otherwise the program
// will continually print A-weighted sound pressure level (SPL dBA)
// and peak sound amplitude (mPa) over the serial port.

#include <stdbool.h>
#include "hardware_profile.h"
#include "print_functions.h"
#include "sound_measurement.h"

bool simplifiedReadout(void);
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

	if (!soundInit(DMA_Init, I2S1_Init, TIM3_Init, DMA1_Channel1_IRQn)) {
		errorHandler(__func__, __LINE__, __FILE__);
	}

	#ifdef TESTS
		test_sound_system();
	#else
		if (!enableMicrophone(true)) {
			errorHandler(__func__, __LINE__, __FILE__);
		}

		while (true) {
			if (simplifiedReadout()) {
				clearMaximumAmplitude(); // Call this at any time
				print("%u.%u  %u.%02u  %u\n", soundData.SPL_dBA_int,
						soundData.SPL_dBA_fr_1dp, soundData.peak_amp_mPa_int,
						soundData.peak_amp_mPa_fr_2dp, soundData.stable);
			}
		}
	#endif
	return 0;
}

// Return bool: new data available
bool simplifiedReadout(void) {
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
		getSoundData(&soundData, true, true);
		enableSPLcalculation(false); // reset the calculation, ready for restart
		stage = 0;
		return true;
	}

	errorHandler(__func__, __LINE__, __FILE__);
	stage = 0;
	return false;
}


