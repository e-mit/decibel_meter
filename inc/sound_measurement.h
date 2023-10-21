#ifndef SOUND_MEASUREMENT_H
#define SOUND_MEASUREMENT_H

#include <stdbool.h>
#include "project_config.h"

#define BIT_ROUNDING_MARGIN 4

// Interrupt priority.
// Priority must be a number 0-3; M0+ does not use subpriorities.
// Equal priority interrupts do not interrupt each other. Lower priorities interrupt higher ones.
// If two equal-priority interrupts are pending, the IRQn breaks the tie.
// NB: Systick interrupt priority is 0 with IRQn = -1
#define DMA_IRQ_PRIORITY 2  // IRQn = 9

#if (I2S_AUDIOFREQ == I2S_AUDIOFREQ_16K)
	#define I2S_FREQ 15625 // the actual value
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_32K)
	#define I2S_FREQ 31250 // the actual value
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_48K)
	#define I2S_FREQ 50000 // the actual value
#else
	#error "Undefined I2S AUDIO FREQ"
#endif

#define EIGHTH_BUFLEN FFT_N
#define HALF_BUFLEN (EIGHTH_BUFLEN*4)
#define FULL_BUFLEN (HALF_BUFLEN*2)

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

bool soundInit(void);
bool enableMicrophone(bool bEnable);
void clearMaximumAmplitude(void);
void enableSPLcalculation(bool bEnable);
volatile bool isSPLcalcComplete(void);
void getSoundData(SoundData_t * data, bool getSPLdata, bool getMaxAmpData);

//////////////////////////////////////////////////////////////////////

#define TMR3_RES_FREQ_KHZ 1  // sets resolution
#define TMR3_PERIOD_MS 1500
#define TMR3_PERIOD ((TMR3_RES_FREQ_KHZ*TMR3_PERIOD_MS)-1)
#define TMR3_PRESCALER ((SYSCLK_FREQ_HZ/(1000*TMR3_RES_FREQ_KHZ))-1)

//////////////////////////////////////////////////////////////////////

// found the following settling period by considering settling time of the IIR filter.
// assumes that the startup period of the mic has already ended (can be ~1.5 seconds).
// the timeout is the time for receiving a half DMA buffer of data, plus a 20% safety margin, then rounded up to nearest ms
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
