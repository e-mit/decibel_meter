#ifndef SOUND_MEASUREMENT_H
#define SOUND_MEASUREMENT_H

#include <stdbool.h>
#include "stm32g0xx_hal.h"

// Sound settings
#define MIC_SETTLING_PERIOD_MS 1500
#define FFT_N 128
#define I2S_AUDIOFREQ I2S_AUDIOFREQ_16K // can be 16, 32, 48
#define FILTER_SPL // if defined: SPL is averaged over N readings, then SPL calc stops.
				   // if not defined: SPL is continuously calculated on each DMA interrupt
				   // and can be read at any time.
#define FILTER_SPL_N 20 // how many consecutive SPL calculations to average over.
						// NOTE: this is not a moving average: Accumulate N readings
						// and average, then start again.

////////////////////////////////////////////////////////

// Frequency bands for sound level measurement
#define SOUND_FREQ_BANDS 6
static const uint16_t sound_band_mids_Hz[SOUND_FREQ_BANDS] = {125, 250, 500, 1000, 2000, 4000};
static const uint16_t sound_band_edges_Hz[SOUND_FREQ_BANDS+1] = {88, 177, 354, 707, 1414, 2828, 5657};

///////////////////////////////////////////////////////////

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
volatile bool isSPLcalcComplete(void);
void getSoundData(SoundData_t * data, bool getSPLdata, bool getMaxAmpData);

//////////////////////////////////////////////////////////////////////

// Define the exact I2S frequency
#if (I2S_AUDIOFREQ == I2S_AUDIOFREQ_16K)
	#define I2S_FREQ 15625
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_32K)
	#define I2S_FREQ 31250
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_48K)
	#define I2S_FREQ 50000
#else
	#error "Unknown I2S AUDIO FREQ"
#endif

#define EIGHTH_BUFLEN FFT_N
#define HALF_BUFLEN (EIGHTH_BUFLEN*4)
#define FULL_BUFLEN (HALF_BUFLEN*2)

// Find the time period for the settling of the amplitude filter, as
// a multiple of the half-DMA buffer fill time.
#if (I2S_FREQ == 31250)
	#if (FFT_N == 256)
		#define N_AMP_SETTLE_HALF_PERIODS 10
	#elif (FFT_N == 512)
		#define N_AMP_SETTLE_HALF_PERIODS 5
	#elif (FFT_N == 1024)
		#define N_AMP_SETTLE_HALF_PERIODS 3
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif
#elif (I2S_FREQ == 15625)
	#if (FFT_N == 128)
		#define N_AMP_SETTLE_HALF_PERIODS 10
	#elif (FFT_N == 256)
		#define N_AMP_SETTLE_HALF_PERIODS 5
	#elif (FFT_N == 512)
		#define N_AMP_SETTLE_HALF_PERIODS 3
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif
#else
	#error("N-points and/or Fs not implemented yet")
#endif

#endif
