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

volatile SoundData_t soundData;
volatile SoundData_t soundData_internal;

#ifdef DEBUG_PRINT
	void printAllData(void) {
		printSerial("SPL = %u.%u dBA\n", soundData.SPL_dBA_int, soundData.SPL_dBA_fr_1dp);
		printSerial("Bands SPL: ");
		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			printSerial("%u.%u ", soundData.SPL_bands_dB_int[i], soundData.SPL_bands_dB_fr_1dp[i]);
		}
		printSerial("dB\n");
		printSerial("MAX amplitude = %u.%02u mPa\n", soundData.peak_amp_mPa_int, soundData.peak_amp_mPa_fr_2dp);
		printSerial("---------\n");
	}
#endif

void clearAllData(void) {
	// clear data buffer:
	memset((void *) &soundData, 0, sizeof (SoundData_t));
	// reinstate mic stabilization status
	soundData.stable = micHasStabilized() ? 1 : 0;
}

void clearAllData_internal(void) {
	// clear data buffer:
	memset((void *) &soundData_internal, 0, sizeof (SoundData_t));
	// reinstate mic stabilization status
	soundData_internal.stable = micHasStabilized() ? 1 : 0;
}

void enterCycleMode(void) {
	clearAllData();
}


///////////////////////////////////////////////////////////////////////////
// sound functions

void maxAmpFollowerOn(bool bEnable) {
	if (!isMicEnabled()) {
		// reject command: no change to register
		return;
	}

	if (bEnable) {
		enable_I2S_DMA_interrupts(true);
		clearMaxAmpFollower(); // in case DMA ints were already enabled
	}
	else {
		// disable DMA interrupts only if sound interrupts are not enabled:
		enable_I2S_DMA_interrupts(false);
	}
}

bool micOn(bool bEnable) {
	if (bEnable) {
		if (!enableMic(true)) {
			return false;
		}
		maxAmpFollowerOn(true);
	}
	else {
		// turning off mic means also disable maxamp
		maxAmpFollowerOn(false);
		if (!enableMic(false)) {
			return false;
		}
	}
	return true;
}


bool applyDefaultSettings(void) {
	bool ok = micOn(true);
	if (false) {
		// ensure DMA is turned off: may have been turned on temporarily for SPL calc:
		enable_I2S_DMA_interrupts(false);
	}
	// in all cases, want SPL disabled:
	enableSPLcalculation(false);

	clearAllData();
	return ok;
}



// Each call attempts to execute one extra stage; the returned value is the
// expected wait in ms until the next stage can execute -> can choose to wait for this long externally OR
// can repeatedly call this function until it progresses.
// Normally have bRestart = true if this is the first cycle after entering cycle mode.
uint32_t staged_IAQ_readout(void) {
	static uint32_t stage = 0;
	static bool readSPL;
	static bool readMaxAmp;

	static SpecifiedDelay_t IAQ_cycle_delay = {0};
	static SpecifiedDelay_t soundAcquisition = {0};

	if (stage == 0) {
		// prepare measurement requests:
		readSPL    =  true;
		readMaxAmp =  true;

		clearSpecifiedDelay(&IAQ_cycle_delay);
		clearSpecifiedDelay(&soundAcquisition);

		stage++;
		return 0;
	}
	else if (stage == 1) { // loop back here, not to stage 0, once this cycle has completed
		// check that the new cycle can start:
		uint32_t timeLeft_ms = getRemainingDelay_ms(&IAQ_cycle_delay);
		if (IAQ_cycle_delay.inProgress) {
			// still waiting for enough time to elapse since previous cycle
			return timeLeft_ms;
		}
		clearAllData_internal();
	}
	else if (stage == 2) {

		// see whether to enable SPLcalc and/or DMA interrupts.
		// Note that if I2S clock is not active, no sound measurements are taken.
		if (isMicEnabled()) {
			if (readMaxAmp && (!isDMAIntEnabled())) {
				readMaxAmp = false; // max amplitude is not being followed: ignore request to measure it
			}
			if (readSPL) {
				enable_I2S_DMA_interrupts(true); // no effect if already on
				enableSPLcalculation(true); // this also resets the sound filter (if used) and clears SPL semaphore
				soundAcquisition.startTime_tick = HAL_GetTick();
				soundAcquisition.duration_ms = EXPECTED_SPL_ACQ_PERIOD_MS;
				soundAcquisition.inProgress = true;
				// now wait for the semaphore to be set true OR timeout to elapse before getting data
			}
		}
		else {
			// I2S clock not active, so no sound measurements are taken.
			readSPL = false;
			readMaxAmp = false;
		}
		stage++;
		return 0;
	}
	else if (stage == 7) {
		// read sound data:
		if (readSPL||readMaxAmp) {

			if (readSPL) {
				// check that enough time has passed: check semaphore but also impose timeout
				// don't do this for maxamp because it is continuously updated
				uint32_t timeLeft_ms = getRemainingDelay_ms(&soundAcquisition);
				if ((!is_SPL_calc_complete()) && (soundAcquisition.inProgress)) {
					return timeLeft_ms;
				}
			}


			SoundData_t sd = {0};
			uint32_t maxAmp_DN; // unused here
			getSoundDataStruct(&sd, readSPL, readMaxAmp, &maxAmp_DN);

			if (readMaxAmp) {
				clearMaxAmpFollower();
			}

			// MAYBE need to do memcpy due to arrays in the struct and worry about mem alignment
			memcpy((void *) &(soundData_internal), (void *) &sd, sizeof (SoundData_t));

			// reinstate previous settings:
			bool leave_DMA_on = true;
			enable_I2S_DMA_interrupts(leave_DMA_on);
			enableSPLcalculation(false);
		}

		stage++;
		return 0;
	}
	else if (stage == 10) {

		// now copy data buffer:
		memcpy((void*)&soundData, (void*)&soundData_internal, sizeof (SoundData_t));

		stage++;
		return 0;
	}


	// reaching here is an error: reset the cycles:
	PRINT("Cycle error: setting stage = 0.\n");
	stage = 0; // restart as if just entering cycle mode
	return 0;
}


