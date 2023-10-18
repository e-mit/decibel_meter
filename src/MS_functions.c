#include "MS_functions.h"
#include "sensor_constants.h"
#include "UART.h"
#include "print_functions.h"
#include "interrupts.h"
#include "hardware_profile.h"
#include "project_config.h"
#include "sound_measurement.h"
#include "utilities.h"
#include "stm32g0xx_hal.h"

void printAllData(SoundData_t * soundData) {
	printSerial("SPL = %u.%u dBA\n", soundData->SPL_dBA_int, soundData->SPL_dBA_fr_1dp);
	printSerial("Bands SPL: ");
	for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
		printSerial("%u.%u ", soundData->SPL_bands_dB_int[i], soundData->SPL_bands_dB_fr_1dp[i]);
	}
	printSerial("dB\n");
	printSerial("MAX amplitude = %u.%02u mPa\n", soundData->peak_amp_mPa_int, soundData->peak_amp_mPa_fr_2dp);
	printSerial("---------\n");
}

