#ifndef SOUND_MEASUREMENT_H
#define SOUND_MEASUREMENT_H

#include "arm_math.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "project_config.h"
#include "sensor_constants.h"

extern volatile bool sound_DMA_semaphore; // set true at end of every DMA ISR

// this is set true by the sound module when a new SPL reading is ready. This may be after every DMA interrupt
// OR (if filter is enabled), after every N DMA interrupts.
// also see function reset_SPL_semaphore();
__attribute__((always_inline)) inline bool is_SPL_calc_complete(void) {
	extern volatile bool SPL_calc_complete;
	// set true after every SPL calculation, which may be on every DMA ISR
	// OR every N ISRs, depending on filter settings.
	return (bool) SPL_calc_complete;
}
void reset_SPL_semaphore(void);

__attribute__((always_inline)) inline bool isMicEnabled(void) {
	extern volatile bool micEnabled;
	return (bool) micEnabled;
}

__attribute__((always_inline)) inline bool isDMAIntEnabled(void) {
	extern volatile bool DMAintEnabled;
	return (bool) DMAintEnabled;
}

__attribute__((always_inline)) inline bool micHasStabilized(void) {
	extern volatile bool micStable;
	return ((bool) micStable);// true means finished warming up
}

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
// User functions:

bool sound_init(void);
bool enableMic(bool bEnable);            // starts/stops I2S clock
void pause_DMA_interrupts(bool bPause);
void enable_I2S_DMA_interrupts(bool bEnable); // starts DMA interrupts and maxAmp following
void enableSPLcalculation(bool bEnable);
void clearMaxAmpFollower(void); // on next cycle, will reset the max amp follower
void getSoundDataStruct(SoundData_t * data, bool getSPLdata, bool getMaxAmpData);
void amplitude_DN_to_mPa(uint32_t ampDN, uint16_t * intAmp_mPa, uint8_t * frac2dpAmp_mPa);
uint32_t amplitude_mPa_to_DN(uint16_t intAmp_mPa);

#ifdef DEBUG_AND_TESTS
	#define NTIMES (4*2)
	#define N_SPL_SAVE 250
	extern volatile uint32_t nspl;
	extern volatile int32_t SPL_intBuf[];
	bool soundUnitTests(void);
	void getSoundTimeData(uint32_t * timesArray);
	int32_t * stopI2S_afterNhalfBuffers(void);
	extern volatile bool autoStopI2S;
	extern volatile uint32_t NhalfBuffersCmpltd, NhalfBufLimit;
#endif

//////////////////////////////////////////////////////////////////////
// non-user functions

void DMA1_Channel1_IRQHandler(void);
void TIM3_IRQHandler(void);

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
		#define SOUND_DMA_PERIOD_TIMEOUT_MS 10
	#elif (FFT_N == 512)
		#define N_AMP_SETTLE_HALF_PERIODS 5
		#define SOUND_DMA_PERIOD_TIMEOUT_MS 20
	#elif (FFT_N == 1024)
		#define N_AMP_SETTLE_HALF_PERIODS 3
		#define SOUND_DMA_PERIOD_TIMEOUT_MS 40
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif
#elif (I2S_FREQ == 15625)
	#if (FFT_N == 128)
		#define N_AMP_SETTLE_HALF_PERIODS 10
		#define SOUND_DMA_PERIOD_TIMEOUT_MS 12
	#elif (FFT_N == 256)
		#define N_AMP_SETTLE_HALF_PERIODS 5
		#define SOUND_DMA_PERIOD_TIMEOUT_MS 24
	#elif (FFT_N == 512)
		#define N_AMP_SETTLE_HALF_PERIODS 3
		#define SOUND_DMA_PERIOD_TIMEOUT_MS 48
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif
#else
	#error("N-points and/or Fs not implemented yet")
#endif

// the following can be used as timeouts; they have margin
#ifdef FILTER_SPL
#define EXPECTED_SPL_ACQ_PERIOD_MS SOUND_DMA_PERIOD_TIMEOUT_MS*(FILTER_SPL_N+1)
#else
#define EXPECTED_SPL_ACQ_PERIOD_MS SOUND_DMA_PERIOD_TIMEOUT_MS*2
#endif

extern volatile uint32_t countCopy;

#endif
