#ifndef SOUND_MEASUREMENT_H
#define SOUND_MEASUREMENT_H

#include "arm_math.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "project_config.h"
#include "sensor_constants.h"

#define BIT_ROUNDING_MARGIN 4

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
// User interface functions:

bool sound_init(void);
bool enableMicrophone(bool bEnable);
void clearMaximumAmplitude(void);
void enableSPLcalculation(bool bEnable);
volatile bool isSPLcalcComplete(void);
void getSoundDataStruct(SoundData_t * data, bool getSPLdata, bool getMaxAmpData);

//////////////////////////////////////////////////////////////////////

#ifdef DEBUG_AND_TESTS
	#define NTIMES (4*2)
	#define N_SPL_SAVE 250
	extern volatile uint32_t nspl;
	extern volatile int32_t * SPL_intBuf;
	bool soundUnitTests(void);
	void getSoundTimeData(uint32_t * timesArray);
	int32_t * stopI2S_afterNhalfBuffers(void);
	extern volatile bool autoStopI2S;
	extern volatile uint32_t NhalfBuffersCmpltd, NhalfBufLimit;
#endif

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
