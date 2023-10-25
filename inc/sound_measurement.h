// Convert I2S data from a microphone into A-weighted Sound Pressure Level (SPL) measurements.
// This code is ARM/STM32-specific but is not limited to a particular device.
//
// Brief explanation of use:
// 1) Initialise I2S, DMA and timer peripherals externally, then pass their handles to soundInit().
// 2) Call enableMicrophone() to start the sound detection and processing.
// 3) Use enableSPLcalculation(), isSPLcalcComplete() and getSoundData() to start/stop, monitor
//    and obtain output data.
// 4) clearMaximumAmplitude() is used to reset the peak amplitude (output in addition to SPL).

#ifndef SOUND_MEASUREMENT_H
#define SOUND_MEASUREMENT_H

#include <stdbool.h>
#include "stm32g0xx_hal.h"
#include "sound_LUTs.h"

// Sound settings
#define FFT_N 128   // FFT points; can be 128 - 1024
#define I2S_AUDIOFREQ I2S_AUDIOFREQ_16K  // Can be 16, 32, 48
#define FILTER_SPL // if defined: SPL is averaged over N readings, then SPL calc stops.
				   // if not defined: SPL is continuously calculated on each DMA interrupt
				   // and can be read at any time.
#define FILTER_SPL_N 20 // how many consecutive SPL calculations to average over.
						// NOTE: this is not a moving average: Accumulate N readings
						// and average, then start again.

////////////////////////////////////////////////////////

typedef struct __attribute__((packed)) {
  uint8_t  SPL_dBA_int;
  uint8_t  SPL_dBA_fr_1dp;
  uint8_t  SPL_bands_dB_int[SOUND_FREQ_BANDS];
  uint8_t  SPL_bands_dB_fr_1dp[SOUND_FREQ_BANDS];
  uint16_t peak_amp_mPa_int;
  uint8_t  peak_amp_mPa_fr_2dp;
  uint8_t  stable;
} SoundData_t;

////////////////////////////////////////////////////////
// User interface functions:

bool soundInit(void (*DMAInit)(DMA_HandleTypeDef **), bool (*I2SInit)(I2S_HandleTypeDef **),
		       bool (*tmrInit)(TIM_HandleTypeDef **), IRQn_Type DMAChIRQn);
bool enableMicrophone(bool bEnable);
void clearMaximumAmplitude(void);
void enableSPLcalculation(bool bEnable);
bool isSPLcalcComplete(void);
void getSoundData(SoundData_t * data, bool getSPLdata, bool getMaxAmpData);

#ifdef TESTS
void test_soundSystem(void);
#endif

#endif
