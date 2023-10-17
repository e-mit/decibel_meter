#include <stdint.h>
#include <stdlib.h>
#include "print_functions.h"
#include "arm_math.h"
#include "sound_measurement.h"
#include "arm_const_structs.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "hardware_profile.h"
#include "sound_LUTs.h"
#include "sound_test_data.h"
#include <string.h>
#include "efficient_10log10.h"
#include "math.h"
#include "utilities.h"
#include <stddef.h>
#include "sensor_constants.h"

// global state vars:
// micEnabled is not declared static so it can be accessed via inline function BUT must
// not be declared extern elsewhere, to maintain encapsulation.
volatile bool sound_DMA_semaphore = false; // set true at end of every DMA ISR
volatile bool SPL_calc_complete = true; // set true after every SPL calculation, which may be on every DMA ISR
										// OR every N ISRs, depending on filter settings.

////////////////////////////////////////////////

// private state variables (static or read through inline func):
static volatile bool SPL_calc_enabled = false;
volatile bool micEnabled = false; // whether it is being clocked
volatile bool DMAintEnabled = false; // whether interrupts can fire
volatile bool micStable = false; // goes false when warmup time is complete

#ifdef FILTER_SPL
static volatile int32_t spl_int_sum = 0;
static volatile int32_t spl_frac1dp_sum = 0;
static volatile int32_t band_spl_int_sum[SOUND_FREQ_BANDS] = {0};
static volatile int32_t band_spl_frac1dp_sum[SOUND_FREQ_BANDS] = {0};
static volatile uint32_t spl_sum_count = 0;
volatile uint32_t countCopy = 0;
#endif

// variables as integer-fraction representation:
static volatile int32_t SPL_int = 0, SPL_frac_1dp = 0;
static volatile int32_t bandSPL_int[SOUND_FREQ_BANDS] = {0}, bandSPL_frac_1dp[SOUND_FREQ_BANDS] = {0};

static volatile uint32_t maxAmp_follower = 0; // stores running maximum until cleared by user

// other data:
static volatile uint16_t dmaBuffer[FULL_BUFLEN] = {0}; // holds raw uint16s received from I2S module, used in 2 halves
static volatile int32_t dataBuffer[EIGHTH_BUFLEN] = {0}; // holds final 24-bit sound data from half of the DMA
I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;
static TIM_HandleTypeDef htim3; // warmup timer

// local array used as input and output to FFT:
static q31_t FFTdata[2*FFT_N] = {0}; // interleaved complex, so need 2x number of elements.

// counter used to ignore the max amplitude for the first N_AMP_SETTLE_PERIODS half-DMA interrupts
// allowing the digital filter to settle.
static volatile uint32_t amplitudeSettlingPeriods = 0;

// flag used to clear maxAmp_follower at appropriate moment of cycle
static volatile bool clearMaxAmpFollowerFlag = false;

#ifdef DEBUG_AND_TESTS
	static volatile float32_t SPL = 0.0;
	static volatile float32_t bandSPL[SOUND_FREQ_BANDS] = {0.0};
	volatile uint32_t timeA_us[NTIMES/2] = {0};
	volatile uint32_t timeB_us[NTIMES/2] = {0};
	volatile uint32_t NhalfBuffersCmpltd = 0, NhalfBufLimit = 0;
	volatile bool autoStopI2S = false;
	volatile uint32_t nspl = 0;
	volatile int32_t SPL_intBuf[N_SPL_SAVE] = {0}; // this will save the first N_SPL_SAVE SPL values
#endif

//////////////////////////////////////////////////////////////////////////////

static void DMA_Init(void);
static bool I2S1_Init(void);
static bool startWarmupPeriod(void);
static void TIM3_Init(void);
static inline void clearAmpFollowerInternal(void);
static void decodeI2SdataLch(const uint16_t inBuf[], const uint32_t inBuflen, int32_t outBuf[]);
static void findMinMax(int32_t * min, int32_t * max, const int32_t array[], const uint32_t length);
static uint32_t getPo2factor(uint32_t bigVal, uint32_t smallVal);
#ifdef DEBUG_AND_TESTS
	static uint32_t getFilteredMaxAmplitude(const int32_t data[], const uint32_t length);
	static void calculateSPL(void);
#endif
static void calculateSPLfast(void);
static uint32_t getFilteredMaxAmplitudeQ31(const int32_t data[], const uint32_t length,
										   bool reset, bool updateMaxAmpFollower);

//////////////////////////////////////////////////////////////////////////////

// read and copy all of the output data, during a brief period of disabled DMA interrupt.
// internal data are never reset to zero, so will read last data if DMAint are disabled. Use the input
// bools to prevent this and return zeros if desired.
void getSoundDataStruct(SoundData_t * data, bool getSPLdata, bool getMaxAmpData, uint32_t * pMaxAmp_DN) {
	if (DMAintEnabled) {
		NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	}
	// need memory barrier instructions here just in case DMA interrupt had already been triggered
	// and would execute in next few cycles. NB: DMB is not needed.
	__DSB();
	__ISB();
	// At this point, know that no DMA ISR is in progress and that it will not trigger until re-enabled.

	if (getSPLdata) {

	#ifdef FILTER_SPL

		countCopy = spl_sum_count;

		if (spl_sum_count == 0) {
			// prevent divide by zero:
			data->SPL_dBA_int = 0;
			data->SPL_dBA_fr_1dp = 0;
			for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
				data->SPL_bands_dB_int[i] = 0;
				data->SPL_bands_dB_fr_1dp[i] = 0;
			}
		}
		else {
			float spl = (((float) spl_int_sum) +
					(((float) spl_frac1dp_sum)/10.0))/((float) spl_sum_count);

			uint32_t intpart = 0;
			uint8_t fracpart1dp = 0;
			float2IntFrac1dp(spl, &intpart, &fracpart1dp);

			if (intpart > UINT8_MAX) {
				data->SPL_dBA_int = UINT8_MAX;
				data->SPL_dBA_fr_1dp = 0;
			}
			else {
				data->SPL_dBA_int = (uint8_t) intpart;
				data->SPL_dBA_fr_1dp = (uint8_t) fracpart1dp;
			}

			for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {

				spl = (((float) band_spl_int_sum[i]) +
						(((float) band_spl_frac1dp_sum[i])/10.0))/((float) spl_sum_count);

				float2IntFrac1dp(spl, &intpart, &fracpart1dp);

				if (intpart > UINT8_MAX) {
					data->SPL_bands_dB_int[i] = UINT8_MAX;
					data->SPL_bands_dB_fr_1dp[i] = 0;
				}
				else {
					data->SPL_bands_dB_int[i] = (uint8_t) intpart;
					data->SPL_bands_dB_fr_1dp[i] = (uint8_t) fracpart1dp;
				}
			}
		}

	#else

		if (SPL_int < 0) {
			data->SPL_dBA_int = 0;
			data->SPL_dBA_fr_1dp = 0;
		}
		else if (SPL_int > UINT8_MAX) {
			data->SPL_dBA_int = UINT8_MAX;
			data->SPL_dBA_fr_1dp = 0;
		}
		else {
			data->SPL_dBA_int = (uint8_t) SPL_int;
			data->SPL_dBA_fr_1dp = (uint8_t) SPL_frac_1dp;
		}

		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			if (bandSPL_int[i] < 0) {
				data->SPL_bands_dB_int[i] = 0;
				data->SPL_bands_dB_fr_1dp[i] = 0;
			}
			else if (bandSPL_int[i] > UINT8_MAX) {
				data->SPL_bands_dB_int[i] = UINT8_MAX;
				data->SPL_bands_dB_fr_1dp[i] = 0;
			}
			else {
				data->SPL_bands_dB_int[i] = (uint8_t) bandSPL_int[i];
				data->SPL_bands_dB_fr_1dp[i] = (uint8_t) bandSPL_frac_1dp[i];
			}
		}
	#endif
	}

	pMaxAmp_DN[0] = 0;
	if (getMaxAmpData) {
		uint32_t maxAmpDN = maxAmp_follower;
		pMaxAmp_DN[0] = maxAmp_follower;
		uint16_t i = 0;
		uint8_t f = 0;
		amplitude_DN_to_mPa(maxAmpDN, &i, &f);
		data->peak_amp_mPa_int = i;
		data->peak_amp_mPa_fr_2dp = f;
	}

	if (micStable) {
		data->stable = 1;
	}
	else {
		data->stable = 0;
	}

	if (DMAintEnabled) {
		NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	}
	// NOTE that any pending DMA interrupt will now fire, but will take ~2 cycles to start
}

void DMA1_Channel1_IRQHandler(void) {
#ifdef DEBUG_AND_TESTS
	HAL_GPIO_WritePin(TEST1_OUTPUT_GPIO_Port, TEST1_OUTPUT_Pin, GPIO_PIN_SET);
#endif
	HAL_DMA_IRQHandler(&hdma_spi1_rx);
#ifdef DEBUG_AND_TESTS
	HAL_GPIO_WritePin(TEST1_OUTPUT_GPIO_Port, TEST1_OUTPUT_Pin, GPIO_PIN_RESET);
#endif
}

bool sound_init(void) {
	DMA_Init();
	TIM3_Init();
	return I2S1_Init();
}

static bool I2S1_Init(void) {
	hi2s1.Instance = SPI1;
	hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
	hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
	hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s1.Init.AudioFreq = I2S_AUDIOFREQ;
	hi2s1.Init.CPOL = I2S_CPOL_LOW;
	return (HAL_I2S_Init(&hi2s1) == HAL_OK);
}

static void DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();

	#if ((DMA_IRQ_PRIORITY > 3)||(DMA_IRQ_PRIORITY<0))
		#error("Interrupt priority must be 0-3")
	#endif

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, DMA_IRQ_PRIORITY, 0);
  // note that the interrupt is not enabled here.
}

static inline void clearAmpFollowerInternal(void) {
	maxAmp_follower = 0;
	clearMaxAmpFollowerFlag = false;
}

static bool startWarmupPeriod(void) {
	// prepare and start a one-shot timer that, on expiry, sets micStable = true and then turns itself off.
	__HAL_TIM_SetCounter(&htim3,0);
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		return false;
	}
	return true;
}

void TIM3_IRQHandler(void) {
	micStable = true;
	HAL_TIM_IRQHandler(&htim3);
	HAL_TIM_Base_Stop_IT(&htim3);
	clearMaxAmpFollower();
}

static void TIM3_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	htim3.Instance = TIM3;
	#if (TMR3_PRESCALER > 65535)
		#error("TMR3 prescaler must be a 16-bit number")
	#endif
	htim3.Init.Prescaler = TMR3_PRESCALER;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	#if (TMR3_PERIOD > 65535)
		#error("TMR3 period must be a 16-bit number")
	#endif
	htim3.Init.Period = TMR3_PERIOD;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler(__func__, __LINE__, __FILE__);
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler(__func__, __LINE__, __FILE__);
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler(__func__, __LINE__, __FILE__);
	}

	// set priority but do not enable yet - may not be used
	#if ((TMR3_IRQ_PRIORITY > 3)||(TMR3_IRQ_PRIORITY<0))
		#error("Interrupt priority must be 0-3")
	#endif
	HAL_NVIC_SetPriority(TIM3_IRQn, TMR3_IRQ_PRIORITY, 0);

	// Note that initialising the time base causes the UIF flag to get set. Clear it.
	// NB: TIM_FLAG_UPDATE == TIM_SR_UIF. "Update" means rollover.
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
}

// enable: starts the I2S clock and starts warmup timer (no DMA interrupt enabling)
// disable: stops the DMA interrupts and stops I2S clock.
bool enableMic(bool bEnable) {
	if (bEnable == micEnabled) {
		return true;
	}
	if (bEnable) {
		// start the I2S clock and start the warmup timer, but do not start the DMA interrupts (separate)
		startWarmupPeriod();
		// NB: use HALF_BUFLEN here because it is the number of I2S samples, not uint16s
		if (HAL_I2S_Receive_DMA(&hi2s1, (uint16_t *) dmaBuffer, HALF_BUFLEN) != HAL_OK) {
			return false;
		}
		micEnabled = true;
		micStable = false;
	}
	else {
		// stop I2S clock and stop DMA interrupts (if not already done)
		enable_I2S_DMA_interrupts(false); // disable DMA interrupts
		enableSPLcalculation(false);
		if (HAL_I2S_DMAStop(&hi2s1) != HAL_OK) {
			return false;
		}
		micEnabled = false;
		micStable = false;
	}
	return true;
}

// use this only temporarily
void pause_DMA_interrupts(bool bPause) {

	/*
	// need memory barrier instructions here just in case DMA interrupt had already been triggered
	// and would execute in next few cycles. NB: DMB is not needed.
	if (bPause) {
		NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	}
	else {
		NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	}
	__DSB();
	__ISB();
	// At this point, know that no DMA ISR is in progress and that it will not trigger until re-enabled.
	 * */

	if (bPause) {
		__HAL_RCC_DMA1_CLK_DISABLE();
		NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	}
	else {
		__HAL_RCC_DMA1_CLK_ENABLE();
		NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	}
	__DSB();
	__ISB();
}

void enable_I2S_DMA_interrupts(bool bEnable) {
	if (bEnable == DMAintEnabled) {
		return;
	}
	if (bEnable) {
		// enable the interrupts and also begin a new digital filter settling period
		// for the amplitude measurement, during which the maxamp
		// measurement (and thus ampl interrupt) are disabled (read ampl as 0). Also reset maxAmp value.
		amplitudeSettlingPeriods = 0;
		clearAmpFollowerInternal();
		// first clear the interrupt flags:
		__HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1 | DMA_ISR_GIF1);
		NVIC_EnableIRQ(DMA1_Channel1_IRQn);
		DMAintEnabled = true;
	}
	else {
		// just disable interrupts; no other clearing or state changes.
		DMAintEnabled = false;
		NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	}
}

// convert input raw I2S data into signed 32 bit numbers, assuming the I2S data is Left
// channel only and the first datum starts at element 0.
// inBuflen is simply the number of elements in inBuf
static void decodeI2SdataLch(const uint16_t inBuf[], const uint32_t inBuflen, int32_t outBuf[]) {
	uint32_t outCount = 0;
	for (uint32_t i = 0; i < inBuflen; i+=4) {
		// join MS16bits and LS16bits, then shift the result down 8 bits because it is a 24-bit
		// value, rather than a 32-bit one. NOTE: it MUST be done like this to get the correct
		// sign bit; cannot do it like the line below.
		outBuf[outCount] = ((int32_t) ((((uint32_t) inBuf[i]) << 16) | ((uint32_t) inBuf[i+1]))) >> 8;
		//NO: outBuf[outCount] = (int32_t) ((((uint32_t) inBuf[i]) << 8) | (((uint32_t) inBuf[i+1]) >> 8));
		outCount++;
	}
}

void clearMaxAmpFollower(void) {
	clearMaxAmpFollowerFlag = true;
}


void enableSPLcalculation(bool bEnable) {
	if (bEnable) {
		reset_SPL_semaphore();
	}
	SPL_calc_enabled = bEnable;
}


// DMA buffer is half full, i.e. "HALF_BUFLEN" uint16s are in first half of dmaBuffer
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeA_us[0]); // this times since the last ISR call
	RESET_TMR15_AND_FLAG;
#endif
	// decode raw data and copy data out of DMA buffer -> could now start refilling the DMA buffer
	decodeI2SdataLch((uint16_t *) dmaBuffer, HALF_BUFLEN, (int32_t *) dataBuffer);
	if (clearMaxAmpFollowerFlag) {
		clearAmpFollowerInternal();
	}
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeA_us[1]);
	RESET_TMR15_AND_FLAG;
#endif
	// Filter the amplitude, find the maximum, and update maxAmp_follower if necessary:
	if (amplitudeSettlingPeriods == 0) {
		// need to allow the IIR filter to settle. Reset the filter, run it, but do not yet
		// use the maxAmp result to update the global follower OR to operate the interrupt
		getFilteredMaxAmplitudeQ31((int32_t *) dataBuffer, (uint32_t) EIGHTH_BUFLEN, true, false);
		amplitudeSettlingPeriods++;
	}
	else if (amplitudeSettlingPeriods < N_AMP_SETTLE_HALF_PERIODS) {
		// need to allow the IIR filter to settle. Run the filter but do not
		// use the maxAmp result to update the global follower OR to operate the interrupt
		getFilteredMaxAmplitudeQ31((int32_t *) dataBuffer, (uint32_t) EIGHTH_BUFLEN, false, false);
		amplitudeSettlingPeriods++;
	}
	else {
		uint32_t maxAmp = getFilteredMaxAmplitudeQ31((int32_t *) dataBuffer, (uint32_t) EIGHTH_BUFLEN, false, true);
	}
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeA_us[2]);
	RESET_TMR15_AND_FLAG;
#endif
	if (SPL_calc_enabled) {
		// calculate A-weighted SPL, octave bands SPL, update maxSPL_follower if necessary:
		calculateSPLfast();
		#ifdef DEBUG_AND_TESTS
		if (nspl < N_SPL_SAVE) {
			SPL_intBuf[nspl] = SPL_int;
			nspl++;
		}
		#endif
	}
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeA_us[3]);
	RESET_TMR15_AND_FLAG;
	if (autoStopI2S) {
		NhalfBuffersCmpltd++;
		if (NhalfBuffersCmpltd >= NhalfBufLimit) {
			enableMic(false);
		}
	}
#endif
	sound_DMA_semaphore = true;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeB_us[0]); // this times since the last ISR call
	RESET_TMR15_AND_FLAG;
#endif
	// decode raw data and copy data out of DMA buffer -> could now start refilling the DMA buffer
	decodeI2SdataLch((uint16_t *) &(dmaBuffer[HALF_BUFLEN]), HALF_BUFLEN, (int32_t *) dataBuffer);
	if (clearMaxAmpFollowerFlag) {
		clearAmpFollowerInternal();
	}
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeB_us[1]);
	RESET_TMR15_AND_FLAG;
#endif
	// Filter the amplitude, find the maximum, and update maxAmp_follower if necessary:
	if (amplitudeSettlingPeriods == 0) {
		// need to allow the IIR filter to settle. Reset the filter, run it, but do not
		// use the maxAmp result to update the global follower OR to operate the interrupt
		getFilteredMaxAmplitudeQ31((int32_t *) dataBuffer, (uint32_t) EIGHTH_BUFLEN, true, false);
		amplitudeSettlingPeriods++;
	}
	else if (amplitudeSettlingPeriods < N_AMP_SETTLE_HALF_PERIODS) {
		// need to allow the IIR filter to settle. Run the filter but do not
		// use the maxAmp result to update the global follower OR to operate the interrupt
		getFilteredMaxAmplitudeQ31((int32_t *) dataBuffer, (uint32_t) EIGHTH_BUFLEN, false, false);
		amplitudeSettlingPeriods++;
	}
	else {
		uint32_t maxAmp = getFilteredMaxAmplitudeQ31((int32_t *) dataBuffer, (uint32_t) EIGHTH_BUFLEN, false, true);
	}
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeB_us[2]);
	RESET_TMR15_AND_FLAG;
#endif
	if (SPL_calc_enabled) {
		// calculate A-weighted SPL, octave bands SPL, update maxSPL_follower if necessary:
		calculateSPLfast();
		#ifdef DEBUG_AND_TESTS
		if (nspl < N_SPL_SAVE) {
			SPL_intBuf[nspl] = SPL_int;
			nspl++;
		}
		#endif
	}
#ifdef DEBUG_AND_TESTS
	GET_TIME_TMR15(timeB_us[3]);
	RESET_TMR15_AND_FLAG;
	if (autoStopI2S) {
		NhalfBuffersCmpltd++;
		if (NhalfBuffersCmpltd >= NhalfBufLimit) {
			enableMic(false);
		}
	}
#endif
	sound_DMA_semaphore = true;
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
#ifdef DEBUG_AND_TESTS
	HAL_GPIO_WritePin(TEST2_OUTPUT_GPIO_Port, TEST2_OUTPUT_Pin, GPIO_PIN_SET);
	printSerial("HAL_I2S_ErrorCallback() occurred.\n");
	Error_Handler(__func__, __LINE__, __FILE__);
#else
	//maybe reset the I2S module here?
#endif
}

static void findMinMax(int32_t * min, int32_t * max, const int32_t array[], const uint32_t length) {
	max[0] = INT32_MIN;
	min[0] = INT32_MAX;
	for (uint32_t i = 0; i<length; i++) {
		if (array[i] < min[0]) {
			min[0] = array[i];
		}
		if (array[i] > max[0]) {
			max[0] = array[i];
		}
	}
}

// find the largest positive integer bitshift m, such that: smallVal*(2^m) <= bigVal
// there is no checking for erroneous inputs (e.g. bigVal < smallVal).
// This is the largest upward bitshift that can be applied to smallVal such
// that it is still smaller than bigVal.
static uint32_t getPo2factor(uint32_t bigVal, uint32_t smallVal) {
	uint32_t bitShift = 0;
	if (smallVal == 0) { // this is the only situation which could cause an infinite loop (/0)
		return 0;
	}
	while (bigVal >= smallVal) {
		bigVal = bigVal >> 1;
		bitShift++;
	}
	bitShift-=1; // correct for going one shift too far
	return bitShift;
}


#ifdef DEBUG_AND_TESTS

// dataBuffer must contain (at least) FFT_N values
// dataBuffer is used for storage etc throughout this function. Since this is shared by both halves of
// the DMA buffer, this function must complete before it is called again for the other half of
// the DMA data buffer.
static void calculateSPL(void) {

	// provide constants which depend on length of FFT/input sample and Fs
	#if (I2S_FREQ == 50000)
		#if (FFT_N == 256)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len256;
			const uint16_t * sqWsc = sqWsc_Fs50000_256;
			const float32_t tenlog10SF = tenlog10SF_Fs50000_256;
			const uint8_t * bandIDs = bandIDs_Fs50000_256;
		#elif (FFT_N == 512)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len512;
			const uint16_t * sqWsc = sqWsc_Fs50000_512;
			const float32_t tenlog10SF = tenlog10SF_Fs50000_512;
			const uint8_t * bandIDs = bandIDs_Fs50000_512;
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#elif (I2S_FREQ == 31250)
		#if (FFT_N == 128)
			#warning("This choice of FFT_N and I2S_FREQ leads causes NO signal in the lowest octave band")
			// bandSPL = 0 means log(0) but also is bad anyway.
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len128;
			const uint16_t * sqWsc = sqWsc_Fs31250_128;
			const float32_t tenlog10SF = tenlog10SF_Fs31250_128;
			const uint8_t * bandIDs = bandIDs_Fs31250_128;
		#elif (FFT_N == 256)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len256;
			const uint16_t * sqWsc = sqWsc_Fs31250_256;
			const float32_t tenlog10SF = tenlog10SF_Fs31250_256;
			const uint8_t * bandIDs = bandIDs_Fs31250_256;
		#elif (FFT_N == 512)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len512;
			const uint16_t * sqWsc = sqWsc_Fs31250_512;
			const float32_t tenlog10SF = tenlog10SF_Fs31250_512;
			const uint8_t * bandIDs = bandIDs_Fs31250_512;
		#elif (FFT_N == 1024)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len1024;
			const uint16_t * sqWsc = sqWsc_Fs31250_1024;
			const float32_t tenlog10SF = tenlog10SF_Fs31250_1024;
			const uint8_t * bandIDs = bandIDs_Fs31250_1024;
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#elif (I2S_FREQ == 15625)
		#if (FFT_N == 128)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len128;
			const uint16_t * sqWsc = sqWsc_Fs15625_128;
			const float32_t tenlog10SF = tenlog10SF_Fs15625_128;
			const uint8_t * bandIDs = bandIDs_Fs15625_128;
		#elif (FFT_N == 256)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len256;
			const uint16_t * sqWsc = sqWsc_Fs15625_256;
			const float32_t tenlog10SF = tenlog10SF_Fs15625_256;
			const uint8_t * bandIDs = bandIDs_Fs15625_256;
		#elif (FFT_N == 512)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len512;
			const uint16_t * sqWsc = sqWsc_Fs15625_512;
			const float32_t tenlog10SF = tenlog10SF_Fs15625_512;
			const uint8_t * bandIDs = bandIDs_Fs15625_512;
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif

	// find max, min values of the input data
	int32_t max, min;
	findMinMax(&min, &max, (int32_t *) dataBuffer, FFT_N);

	// calc centre of signal range, and the largest bitshift needed to fill the available
	// range without saturating
	int32_t centre = (min/2) + (max/2); //prev used bitshift here -> no
	uint32_t amplitude = (uint32_t) (max - centre + 4); // add 4 for safety margin
	uint32_t bitShift = getPo2factor((uint32_t) INT32_MAX, amplitude);

	// apply offset and bitshift and put data into input vector
	// NOTE: bitshift could be done as Q31, and thus saturating, but this should be unnecessary.
	uint32_t count = 0;
	for (uint32_t i = 0; i<FFT_N; i++) {
		FFTdata[count] = (q31_t) ((dataBuffer[i] - centre) << bitShift); // uses a left-shift on a signed int: appears OK
		FFTdata[count+1] = 0; // MUST do this because the zeros get overwritten later, then array is reused.
		count+=2;
	}

	#if defined(PRINT_TESTS) && defined(DEBUG_AND_TESTS)
		printSerial("Input min = %i, max = %i, centre = %i, amplitude = %i, bitshift = %i\n\n\n",
				     min, max, centre, amplitude, bitShift);
		printSerial("Scaled FFT real input\n");
		for (uint32_t i = 0; i<(2*TEST_LENGTH_SAMPLES); i+=2) {
			printSerial("%i\n", FFTdata[i]);
		}
		printSerial("\n\n\n\n");
	#endif

	// calc FFT
	arm_cfft_q31(&S, FFTdata, 0, 1); // the output is internally divided by N (#points)

	#if defined(PRINT_TESTS) && defined(DEBUG_AND_TESTS)
		printSerial("Raw FFT output\n");
		for (uint32_t i=0; i<(TEST_LENGTH_SAMPLES); i++) {
			printSerial("%i\n", FFTdata[i]);
		}
		printSerial("\n\n\n\n");
	#endif

	// find FFT output max, min values (in 1st half of output), ignoring the two dc bin values:
	findMinMax(&min, &max, &(FFTdata[2]), ((uint32_t) FFT_N) - 2);

	// find the largest absolute real/imag component (store in "max")
	if (min == INT32_MIN) {
		min = INT32_MAX;
	}
	if (max == INT32_MIN) {
		max = INT32_MAX;
	}
	min = abs(min);
	max = abs(max);
	if (min > max) {
		max = min;
	}

	uint32_t amplitude2 = ((uint32_t) max) + 4; // add 4 for safety margin
	uint32_t bitShift2 = getPo2factor((uint32_t) INT32_MAX, amplitude2);

	/// apply the bitshift (not to dc bin, and to 1st half of data only), then get abs mag of each bin
	for (uint32_t i = 2; i<FFT_N; i++) {
		FFTdata[i] = (q31_t) (FFTdata[i] << bitShift2); // uses a left-shift on a signed int: appears OK
	}
	q31_t * sqmag = (q31_t *) dataBuffer; // nb: this re-uses the input dataBuffer as working mem.
	arm_cmplx_mag_squared_q31(FFTdata, sqmag, FFT_N/2); // output is in 3.29 format

	// now apply weighting and sum (but exclude dc bin). Also sum for unweighted octave band SPL:
	uint64_t bandSum[SOUND_FREQ_BANDS] = {0};
	uint64_t sumSq = 0; // sum of squared weighted magnitudes (scaled)
	for (uint32_t i=1; i<(FFT_N/2); i++) {
		sumSq += ((uint64_t)sqmag[i])*((uint64_t) sqWsc[i]);
		if (bandIDs[i] != SOUND_FREQ_BANDS) {
			// this FFT bin DOES belong in one of the octave-freq-bands:
			bandSum[bandIDs[i]] += (uint64_t)sqmag[i];
		}
	}

	// reverse the scalings (which have been applied explicitly and implicitly) using a bitshift
	int32_t bs = ((int32_t) (2*bitShift)) + ((int32_t) (2*bitShift2)) -3 -31; // right shift this amount
	uint32_t absShift = (uint32_t) abs(bs);
	if (bs < 0) {
		// ie a left-shift
		sumSq = (sumSq << absShift);
		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			bandSum[i] = (bandSum[i] << absShift);
		}
	}
	else {
		// right shift
		sumSq = (sumSq >> absShift);
		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			bandSum[i] = (bandSum[i] >> absShift);
		}
	}
	float32_t additiveTerm = -1.5484097e+01;
	/* NB: additiveTerm is constant for a given microphone:
	const = sqrt(2)/(((2^(Nbits-1))-1)*(10^(sig/20))); % = 3.3638e-06
	additiveTerm = 20*log10(const/(20e-6)); % = -1.5484097e+01; */

	float32_t SPLvalue = (10.0*log10((float)sumSq)) + additiveTerm + tenlog10SF;

	for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
		// NB: since the band values are unweighted, there is no "tenlog10SF" term
		bandSPL[i] = (10.0*log10((float) bandSum[i])) + additiveTerm;

	}

	// NOTE: could apply a digital filter to SPL here

	// set the global volatile value:
	SPL = SPLvalue;

	#if defined(PRINT_TESTS) && defined(DEBUG_AND_TESTS)
		printSerial("FFT output max = %i, amplitude2 = %i, bitshift2 = %i\n\n\n", max, amplitude2, bitShift2);

		printSerial("Scaled FFT output:\n");
		for (uint32_t i = 0; i<TEST_LENGTH_SAMPLES; i++) {
			printSerial("%i\n", data[i]);
		}
		printSerial("\n\n\n\n");

		printSerial("Squared magnitude:\n");
		for (uint32_t i=0; i<(TEST_LENGTH_SAMPLES/2); i++) {
			printSerial("%i\n", sqmag[i]);
		}
		printSerial("\n\n\n\n");

		printSerial("sumSq = ");
		printU64hex(sumSq);
		printSerial("\n\n");

		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			printSerial("band %i sumSq = ",i);
			printU64hex(bandSum[i]);
			printSerial(" = %.5f\n",(float) bandSum[i]);
		}

		printSerial("bs = %i, sumSqShifted = %.3e, SPL = : %.3f\n", bs, sumSqShifted, SPLvalue);

	#endif

}
#endif

// with improvements for fast processing (e.g. no floats)
static void calculateSPLfast(void) {

	// provide constants which depend on length of FFT/input sample and Fs
	#if (I2S_FREQ == 50000)
		#if (FFT_N == 256)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len256;
			const uint16_t * sqWsc = sqWsc_Fs50000_256;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs50000_256;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs50000_256;
			const uint8_t * bandIDs = bandIDs_Fs50000_256;
		#elif (FFT_N == 512)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len512;
			const uint16_t * sqWsc = sqWsc_Fs50000_512;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs50000_512;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs50000_512;
			const uint8_t * bandIDs = bandIDs_Fs50000_512;
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#elif (I2S_FREQ == 31250)
		#if (FFT_N == 128)
			#warning("This choice of FFT_N and I2S_FREQ leads causes NO signal in the lowest octave band")
			// bandSPL = 0 means log(0) but also is bad anyway.
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len128;
			const uint16_t * sqWsc = sqWsc_Fs31250_128;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs31250_128;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs31250_128;
			const uint8_t * bandIDs = bandIDs_Fs31250_128;
		#elif (FFT_N == 256)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len256;
			const uint16_t * sqWsc = sqWsc_Fs31250_256;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs31250_256;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs31250_256;
			const uint8_t * bandIDs = bandIDs_Fs31250_256;
		#elif (FFT_N == 512)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len512;
			const uint16_t * sqWsc = sqWsc_Fs31250_512;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs31250_512;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs31250_512;
			const uint8_t * bandIDs = bandIDs_Fs31250_512;
		#elif (FFT_N == 1024)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len1024;
			const uint16_t * sqWsc = sqWsc_Fs31250_1024;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs31250_1024;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs31250_1024;
			const uint8_t * bandIDs = bandIDs_Fs31250_1024;
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#elif (I2S_FREQ == 15625)
		#if (FFT_N == 128)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len128;
			const uint16_t * sqWsc = sqWsc_Fs15625_128;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs15625_128;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs15625_128;
			const uint8_t * bandIDs = bandIDs_Fs15625_128;
		#elif (FFT_N == 256)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len256;
			const uint16_t * sqWsc = sqWsc_Fs15625_256;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs15625_256;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs15625_256;
			const uint8_t * bandIDs = bandIDs_Fs15625_256;
		#elif (FFT_N == 512)
			arm_cfft_instance_q31 S = arm_cfft_sR_q31_len512;
			const uint16_t * sqWsc = sqWsc_Fs15625_512;
			const int32_t tenlog10SF_int = tenlog10SF_int_Fs15625_512;
			const int32_t tenlog10SF_frac = tenlog10SF_frac_Fs15625_512;
			const uint8_t * bandIDs = bandIDs_Fs15625_512;
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif

	// find max, min values of the input data
	int32_t max, min;
	findMinMax(&min, &max, (int32_t *) dataBuffer, FFT_N);

	// calc centre of signal range, and the largest bitshift needed to fill the available
	// range without saturating
	int32_t centre = (min/2) + (max/2); //prev used bitshift here -> no
	uint32_t amplitude = (uint32_t) (max - centre + 4); // add 4 for safety margin
	uint32_t bitShift = getPo2factor((uint32_t) INT32_MAX, amplitude);

	// apply offset and bitshift and put data into input vector
	// NOTE: bitshift could be done as Q31, and thus saturating, but this should be unnecessary.
	uint32_t count = 0;
	for (uint32_t i = 0; i<FFT_N; i++) {
		FFTdata[count] = (q31_t) ((dataBuffer[i] - centre) << bitShift); // uses a left-shift on a signed int: appears OK
		FFTdata[count+1] = 0; // MUST do this because the zeros get overwritten later, then array is reused.
		count+=2;
	}

	#if defined(PRINT_TESTS) && defined(DEBUG_AND_TESTS)
		printSerial("Input min = %i, max = %i, centre = %i, amplitude = %i, bitshift = %i\n\n\n",
				     min, max, centre, amplitude, bitShift);
		printSerial("Scaled FFT real input\n");
		for (uint32_t i = 0; i<(2*TEST_LENGTH_SAMPLES); i+=2) {
			printSerial("%i\n", FFTdata[i]);
		}
		printSerial("\n\n\n\n");
	#endif

	// calc FFT
	arm_cfft_q31(&S, FFTdata, 0, 1); // the output is internally divided by N (#points)

	#if defined(PRINT_TESTS) && defined(DEBUG_AND_TESTS)
		printSerial("Raw FFT output\n");
		for (uint32_t i=0; i<(TEST_LENGTH_SAMPLES); i++) {
			printSerial("%i\n", FFTdata[i]);
		}
		printSerial("\n\n\n\n");
	#endif

	// find FFT output max, min values (in 1st half of output), ignoring the two dc bin values:
	findMinMax(&min, &max, &(FFTdata[2]), ((uint32_t) FFT_N) - 2);

	// find the largest absolute real/imag component (store in "max")
	if (min == INT32_MIN) {
		min = INT32_MAX;
	}
	if (max == INT32_MIN) {
		max = INT32_MAX;
	}
	min = abs(min);
	max = abs(max);
	if (min > max) {
		max = min;
	}

	uint32_t amplitude2 = ((uint32_t) max) + 4; // add 4 for safety margin
	uint32_t bitShift2 = getPo2factor((uint32_t) INT32_MAX, amplitude2);

	/// apply the bitshift (not to dc bin, and to 1st half of data only), then get abs mag of each bin
	for (uint32_t i = 2; i<FFT_N; i++) {
		FFTdata[i] = (q31_t) (FFTdata[i] << bitShift2); // uses a left-shift on a signed int: appears OK
	}
	q31_t * sqmag = (q31_t *) dataBuffer; // nb: this re-uses the input dataBuffer as working mem.
	arm_cmplx_mag_squared_q31(FFTdata, sqmag, FFT_N/2); // output is in 3.29 format

	// now apply weighting and sum, excluding the dc bin. Also sum for unweighted octave band SPL:
	uint64_t bandSum[SOUND_FREQ_BANDS] = {0};
	uint64_t sumSq = 0; // sum of squared weighted magnitudes (scaled)
	for (uint32_t i=1; i<(FFT_N/2); i++) {
		sumSq += ((uint64_t)sqmag[i])*((uint64_t) sqWsc[i]);
		if (bandIDs[i] != SOUND_FREQ_BANDS) {
			// this FFT bin DOES belong in one of the octave-freq-bands:
			bandSum[bandIDs[i]] += (uint64_t)sqmag[i];
		}
	}

	// reverse the scalings (which have been applied explicitly and implicitly) using a bitshift
	int32_t bs = ((int32_t) (2*bitShift)) + ((int32_t) (2*bitShift2)) -3 -31; // right shift this amount
	uint32_t absShift = (uint32_t) abs(bs);
	if (bs < 0) {
		// ie a left-shift
		sumSq = (sumSq << absShift);
		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			bandSum[i] = (bandSum[i] << absShift);
		}
	}
	else {
		// right shift
		sumSq = (sumSq >> absShift);
		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			bandSum[i] = (bandSum[i] >> absShift);
		}
	}

	// Add on terms accounting for microphone parameters
	// and (only for weighted SPL) the weighting scale factor

	/* NB: additiveTerm is constant for a given microphone:
	const = sqrt(2)/(((2^(Nbits-1))-1)*(10^(sig/20))); % = 3.3638e-06
	additiveTerm = 20*log10(const/(20e-6)); % = -1.5484097e+01 = -15.5 to 1.d.p.
	These values assume the mic sensitivity = -26dB
	 */
	const int32_t additiveTerm_int = -15;
	const int32_t additiveTerm_frac = -5;

	// calculate: SPLvalue = (10.0*log10(sumSq)) + additiveTerm + tenlog10SF;
	int32_t SPLintegerPart, SPLfractionalPart;
	efficient_10log10(sumSq, &SPLintegerPart, &SPLfractionalPart);
	SPLintegerPart = SPLintegerPart + additiveTerm_int + tenlog10SF_int;
	SPLfractionalPart = SPLfractionalPart + additiveTerm_frac + tenlog10SF_frac;
	// Apply correction if fractional part is not in range 0->9:
	correctIntFracNumber(&SPLintegerPart, &SPLfractionalPart);
	// copy to the volatile global object:
	SPL_int = SPLintegerPart;
	SPL_frac_1dp = SPLfractionalPart;

	// for the band SPLs just add the mic correction, not the weighting "tenlog10SF" term
	for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
		//bandSPL[i] = (10.0*log10(bandSum[i])) + additiveTerm;

		int32_t intPart, fracPart;
		efficient_10log10(bandSum[i], &intPart, &fracPart);
		bandSPL_int[i] = intPart + additiveTerm_int;
		bandSPL_frac_1dp[i] = fracPart + additiveTerm_frac;
		// Apply correction if fractional part is not in range 0->9:
		correctIntFracNumber((int32_t *) &(bandSPL_int[i]), (int32_t *) &(bandSPL_frac_1dp[i]));
	}

	#ifdef FILTER_SPL
	if (!SPL_calc_complete) {
		spl_int_sum += SPLintegerPart;
		spl_frac1dp_sum += SPLfractionalPart;

		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			band_spl_int_sum[i] += bandSPL_int[i];
			band_spl_frac1dp_sum[i] += bandSPL_frac_1dp[i];
		}

		spl_sum_count++;
		if (spl_sum_count >= FILTER_SPL_N) {
			SPL_calc_complete = true;
			SPL_calc_enabled = false;
		}
	}
	#else
	SPL_calc_complete = true;
	#endif

	#if defined(PRINT_TESTS) && defined(DEBUG_AND_TESTS)
		printSerial("FFT output max = %i, amplitude2 = %i, bitshift2 = %i\n\n\n", max, amplitude2, bitShift2);

		printSerial("Scaled FFT output:\n");
		for (uint32_t i = 0; i<TEST_LENGTH_SAMPLES; i++) {
			printSerial("%i\n", data[i]);
		}
		printSerial("\n\n\n\n");

		printSerial("Squared magnitude:\n");
		for (uint32_t i=0; i<(TEST_LENGTH_SAMPLES/2); i++) {
			printSerial("%i\n", sqmag[i]);
		}
		printSerial("\n\n\n\n");

		printSerial("sumSq = ");
		printU64hex(sumSq);
		printSerial("\n\n");

		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			printSerial("band %i sumSq = ",i);
			printU64hex(bandSum[i]);
			printSerial(" = %.5f\n",(float) bandSum[i]);
		}

		printSerial("bs = %i\n", bs);

	#endif
}

// if filtering, also reset the filter state
void reset_SPL_semaphore(void) {
#ifdef FILTER_SPL
	spl_int_sum = 0;
	spl_frac1dp_sum = 0;

	for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
		band_spl_int_sum[i] = 0;
		band_spl_frac1dp_sum[i] = 0;
	}

	spl_sum_count = 0;
#endif
	SPL_calc_complete = false;
}


#ifdef DEBUG_AND_TESTS

// apply a simple single-pole hi-pass IIR filter to the incoming data to remove the dc offset,
// and return the largest +ve amplitude from this input array. Also update the global max amplitude
// as part of the max-following feature.
static uint32_t getFilteredMaxAmplitude(const int32_t data[], const uint32_t length) {
	static float32_t filtered = 0.0F;
	static float32_t lastData = 0.0F;
	// Want a filter with cutoff of fc = 10Hz. The coeffs will depend on Fs according to:
	// b = exp(-2.pi.(1/Fs).fc)
	// a0 = (1+b)/2
	// a1 = -a0
#if (I2S_AUDIOFREQ == I2S_AUDIOFREQ_16K)
	const float32_t a0 = 0.99799F, a1 = -0.99799F, b = 0.99599F;
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_32K)
	const float32_t a0 = 0.99900F, a1 = -0.99900F, b = 0.99799F;
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_48K)
	const float32_t a0 = 0.99937F, a1 = -0.99937F, b = 0.99874F;
#else
	#error "Undefined I2S AUDIO FREQ"
#endif

	float32_t maxAmp = 0.0F;
	for (uint32_t i=0; i<length; i++) {
		float32_t fx = (float32_t) data[i];
		filtered = (a0*fx) + (a1*lastData) + (b*filtered);
		lastData = fx;
		if (filtered > maxAmp) {
			maxAmp = filtered;
		}
	}
	uint32_t ma32 = (uint32_t) maxAmp;
	if (ma32 > maxAmp_follower) {
		maxAmp_follower = ma32;
	}

	return ma32;
}

#endif

// as for getFilteredMaxAmplitude() but use Q31 operations.
static uint32_t getFilteredMaxAmplitudeQ31(const int32_t data[], const uint32_t length,
										   bool reset, bool updateMaxAmpFollower) {
	static q31_t filtered = 0;
	static q31_t lastData = 0;
	// Want a filter with cutoff of fc = 10Hz. The coeffs will depend on Fs according to:
	// b = exp(-2.pi.(1/Fs).fc)
	// a0 = (1+b)/2
	// a1 = -a0
	// then convert to Q31 by dividing by 2^-31 and then rounding
#if (I2S_AUDIOFREQ == I2S_AUDIOFREQ_16K)
	q31_t a0 = 2143174546, b = 2138865443;
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_32K)
	q31_t a0 = 2145326931, b = 2143170214;
#elif (I2S_AUDIOFREQ == I2S_AUDIOFREQ_48K)
	q31_t a0 = 2146135192, b = 2144786735;
#else
	#error "Undefined I2S AUDIO FREQ"
#endif

	if (reset) {
		// reset the state of the digital filter; e.g. if the mic has been disabled then re-enabled.
		filtered = 0;
		lastData = 0;
	}

	q31_t maxAmp = 0;
	q31_t minAmp = 0;
	// apply a bitshift to the incoming data to maximise the dynamic range BUT while guaranteeing
	// the intermediate value cannot overflow (three Q31 values are added together).
	const uint32_t bitShift = 5;
	for (uint32_t i=0; i<length; i++) {
		q31_t fx = (q31_t) (data[i] << bitShift);

		// to do D = A*B saturating:
		// arm_mult_q31(&A,&B,&D,1);
		// to do D = A+B saturating:
		// arm_add_q31(&A,&B,&D,1);

		// now do: filtered = (a0*fx) + (a1*lastData) + (b*filtered);
		// BUT NOTE that since a1 = -a0 this becomes:
		// filtered = (a0*(fx - lastData)) + (b*filtered)
		q31_t r1, r2, r3;
		lastData = -lastData;
		arm_add_q31(&fx,&lastData,&r1,1); // r1 = fx - lastData
		arm_mult_q31(&a0,&r1,&r2,1);      // r2 = a0*r1
		arm_mult_q31(&b,&filtered,&r3,1); // r3 = b*filtered
		arm_add_q31(&r2,&r3,&filtered,1); // filtered = r2 + r3

		lastData = fx;
		if (filtered > maxAmp) {
			maxAmp = filtered;
		}
		else if (filtered < minAmp) {
			minAmp = filtered;
		}
	}
	uint32_t mn = abs(minAmp);
	uint32_t mp = (uint32_t) maxAmp;
	uint32_t absMaxAmp = 0;
	if (mn > mp) {
		absMaxAmp = mn;
	}
	else {
		absMaxAmp = mp;
	}
	uint32_t ma32 = (uint32_t) (absMaxAmp >> bitShift); // reverse the scaling bitshift

	if (updateMaxAmpFollower && (ma32 > maxAmp_follower)) {
		maxAmp_follower = ma32;
	}

	return ma32;
}


// convert amplitude value in digital numbers to amplitude in milliPascals
// works for peak amplitude, general amplitude, RMS amplitude.
// returns integer and fractional part to 2 d.p.
// Given that ampDN is at most 2^24, the output will alwyas fit in a uint16.
void amplitude_DN_to_mPa(uint32_t ampDN, uint16_t * intAmp_mPa, uint8_t * frac2dpAmp_mPa) {
	/* NOTE: depends on details of microphone
	amp/Pa = (amp/DN)*ik;
	where ik = sqrt(2)/((10^(sig/20))*((2^(NBits-1))-1)) = 3.3638e-6 Pa if sig = -26 dB
	*/
	const float ik_mPa = 3.3638e-3;
	float amp = ((float) ampDN)*ik_mPa;
	uint32_t intpart = 0;
	float2IntFrac2dp(amp, &intpart, frac2dpAmp_mPa);
	intAmp_mPa[0] = (uint16_t) intpart;
}

// convert an integer amplitude mPa to DN
// even the max possible input (UINT16_MAX) will not exceed the output limit (uint32)
uint32_t amplitude_mPa_to_DN(uint16_t intAmp_mPa) {
	/* NOTE: depends on details of microphone
	see also: amplitude_DN_to_mPa()
	(amp/DN) = (amp/mPa)*iik
	*/
	const float iik = 297.28;
	float amp = ((float) intAmp_mPa)*iik;

	return ((uint32_t) amp);
}


#ifdef DEBUG_AND_TESTS

// to get an array (FFT_N values) of amplitude data after a specific number of half-buffers have elapsed:
// 1) MUST disable SPL_calc because this overwrites the dataBuffer array
// 2) set autoStopI2S = true
// 3) set global variable "NhalfBufLimit" to the desired number
// 4) enable I2S.
// 5) Then use this function to check for completion and to get the array of extracted values.
int32_t * stopI2S_afterNhalfBuffers(void) {
	if (NhalfBuffersCmpltd >= NhalfBufLimit) {
		return (int32_t *) dataBuffer;
	}
	else {
		return NULL;
	}
}

// read and copy all of the time data, during a brief period of disabled DMA interrupt.
void getSoundTimeData(uint32_t * timesArray) {
	if (DMAintEnabled) {
		NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	}
	__DSB(); // memory barrier instructions
	__ISB();
	for (uint32_t i=0; i<(NTIMES/2); i++) {
		timesArray[i] = timeA_us[i];
	}
	uint32_t j = 0;
	for (uint32_t i=(NTIMES/2); i<NTIMES; i++) {
		timesArray[i] = timeB_us[j];
		j++;
	}
	if (DMAintEnabled) {
		NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	}
}

bool soundUnitTests(void) {

	#define PC_DIFF_LIMIT 2 // maximum abs() allowable percentage difference
	volatile uint32_t time_us = 0;
	volatile float pcDiff = 0.0;
	volatile float volFloat = 0.0;

	bool testsOK = true, ok = true;

	printSerial("\nSound unit tests on %s for %s\n",__DATE__,__FILE__);
	printSerial("FFT_N = %u, I2S_FREQ = %u\n\n",FFT_N, I2S_FREQ);

	// decodeI2SdataLch: convert input raw I2S data into signed 32 bit numbers, assuming the I2S data is Left
	// channel only and the first datum starts at element 0.
	/*Left channel is received first (sound data are on left channel
	when mic sel pin is low). Thus the first few elements of the data
	buffer end up as:
	Left1H, Left1L, Right1H, Right1L, Left2H, Left2L, Right2H, Right2L,...
	where L/H = low/high 16-bits of the 32-bit data value
	and 1,2,3 = sample number.*/
	#define nsamp 4
	uint16_t inBuf[nsamp*4] = {0x7FF1, 0xE800, 0, 0, 0xB202, 0xB900, 0, 0, 65157, 16384, 0, 0, 1, 1, 0, 0};
	uint32_t inBuflen = nsamp*4;
	int32_t outBuf[nsamp] = {0};
	const int32_t outBufExpected[nsamp] = {8385000, -5111111, -96960, 256};
	decodeI2SdataLch(inBuf, inBuflen, outBuf);
	ok = true;
	for (uint32_t i=0; i<nsamp; i++) {
		//printSerial("EXP: %i, OBT: %i\n",outBufExpected[i], outBuf[i]);
		ok = ok && (outBufExpected[i] == outBuf[i]);
	}
	printSerial("decodeI2SdataLch: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	// findMinMax

	int32_t min, max;
	#define nn 5
	int32_t array[nn] = {(INT32_MIN+2), 99999999, -222222, 0, 56};
	const int32_t maxExpected = 99999999;
	const int32_t minExpected = (INT32_MIN+2);
	findMinMax(&min, &max, array, nn);
	ok = true;
	ok = ok && (max == maxExpected);
	ok = ok && (min == minExpected);
	printSerial("findMinMax: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	//getPo2factor
	// find the largest positive integer bitshift m, such that: smallVal*(2^m) <= bigVal
	// this is the largest upward bitshift that can be applied to smallVal such
	// that it is still <= bigVal.
	// there is no checking for erroneous inputs (e.g. bigVal < smallVal)

	ok = true;
	ok = ok && (getPo2factor(16, 2) == 3);
	ok = ok && (getPo2factor(16, 3) == 2);
	ok = ok && (getPo2factor(16, 0) == 0);
	ok = ok && (getPo2factor(2,  2) == 0);
	ok = ok && (getPo2factor(24, 3) == 3);
	printSerial("getPo2factor: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	//getFilteredMaxAmplitude
	// apply a simple single-pole hi-pass IIR filter to the incoming data to remove the dc offset,
	// and return the largest +ve amplitude from this input array. Called on the decoded I2S values.

	// as input data, use the array x1_32 (length=X1_32_LEN=1024).
	// First filter the initial X1_32_LEN - NFILT values so that the filter state
	// becomes less dependent on the previous memory (although this test function should be run
	// at start of main anyway, so state is the initial state).
	// Then filter the final NFILT values and return the maximum from this batch (and time this too).

	// the expected result depends on Fs and NFILT (but keep the latter constant)
	#define NFILT 128
	#if (I2S_FREQ == 15625)
		const float max_filt_expected = 176936.800155;
	#elif (I2S_FREQ == 31250)
		const float max_filt_expected = 123698.439946;
	#else
		#error("Fs not implemented yet")
	#endif

	volatile uint32_t fma_ignore = getFilteredMaxAmplitude(x1_32, X1_32_LEN - NFILT); // ignore return value
	fma_ignore++; // to prevent unused warning
	RESET_TMR15_AND_FLAG;
	volatile uint32_t filtmax = getFilteredMaxAmplitude(&(x1_32[X1_32_LEN - NFILT]), NFILT);
	GET_TIME_TMR15(time_us);
	printSerial("Time for filtering (float) is: %u us\n",time_us);
	pcDiff = (100.0*(((float) filtmax) - max_filt_expected))/max_filt_expected;
	printSerial("Percentage difference (float) = %.2f%%\n",pcDiff);
	ok = ((pcDiff < PC_DIFF_LIMIT)&&(pcDiff > -PC_DIFF_LIMIT));
	printSerial("getFilteredMaxAmplitude: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	// now do Q31 version

	fma_ignore = getFilteredMaxAmplitudeQ31(x1_32, X1_32_LEN - NFILT, false, true); // ignore return value
	RESET_TMR15_AND_FLAG;
	filtmax = getFilteredMaxAmplitudeQ31(&(x1_32[X1_32_LEN - NFILT]), NFILT, false, true);
	GET_TIME_TMR15(time_us);
	printSerial("Time for filtering (Q31) is: %u us\n",time_us);
	pcDiff = (100.0*(((float) filtmax) - max_filt_expected))/max_filt_expected;
	printSerial("Percentage difference (Q31) = %.2f%%\n",pcDiff);
	ok = ((pcDiff < PC_DIFF_LIMIT)&&(pcDiff > -PC_DIFF_LIMIT));
	printSerial("getFilteredMaxAmplitudeQ31: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	// calculateSPL()

	// input: dataBuffer which must contain FFT_N values
	// dataBuffer must be writable as it is used for storage etc throughout this function.
	// the SPL result is returned and global bandSPL[] is also modified

	// use x1_32 as input data BUT the results also depend on FFT_N and Fs
#if (I2S_FREQ == 31250)
	#if (FFT_N == 128)

	#elif (FFT_N == 256)
		const float SPL_expected = 87.9; //87.909407;
		// {49.18719, 47.61427, 54.74916, 63.08692, 86.66056, 65.99084};
		const float bandSPL_expected[SOUND_FREQ_BANDS] = {49.2, 47.6, 54.7, 63.1, 86.7, 66.0};
	#elif (FFT_N == 512)
		const float SPL_expected = 87.9; //87.947914;
		//{42.72702, 49.50471, 54.34899, 63.27509, 86.69020, 66.20656};
		const float bandSPL_expected[SOUND_FREQ_BANDS] = {42.7, 49.5, 54.3, 63.3, 86.7, 66.2};
	#elif (FFT_N == 1024)
		const float SPL_expected = 88.0; //87.964126;
		// {32.29662, 30.94850, 31.75690, 31.17087, 86.76750, 45.40472};
		const float bandSPL_expected[SOUND_FREQ_BANDS] = {32.3, 30.9, 31.8, 31.2, 86.8, 45.4};
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif
#elif (I2S_FREQ == 15625)
	#if (FFT_N == 128)
		// values calculated with octave on the first 128 values in x1_32:
		const float SPL_expected = 86.7; //86.693684;
		//{52.83744, 54.53367, 62.10349, 86.72742, 62.75779, 56.55911};
		const float bandSPL_expected[SOUND_FREQ_BANDS] = {52.8, 54.5, 62.1, 86.7, 62.8, 56.6};
	#elif (FFT_N == 256)
		const float SPL_expected = 86.7; //86.687506;
		//{47.61427, 54.74916, 63.08692, 86.66056, 65.99084, 61.29402};
		const float bandSPL_expected[SOUND_FREQ_BANDS] = {47.6, 54.7, 63.1, 86.7, 66.0, 61.3};
	#elif (FFT_N == 512)
		const float SPL_expected = 86.8; //86.750352;
		//{49.50471, 54.34899, 63.27509, 86.69020, 66.20656, 61.22843};
		const float bandSPL_expected[SOUND_FREQ_BANDS] = {49.5, 54.3, 63.3, 86.7, 66.2, 61.2};
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif
#else
	#error("N-points and/or Fs not implemented yet")
#endif

	printSerial("calculateSPL() test\n");

	memcpy((int32_t *) &dataBuffer, &x1_32, 4*FFT_N); // 3rd arg is #bytes

	RESET_TMR15_AND_FLAG;
	calculateSPL();
	GET_TIME_TMR15(time_us);
	// convert the result to 1.d.p for a fair test:
	SPL = roundf(SPL*10.0)/10.0;

	printSerial("	Time taken for SPL calc is: %u us\n",time_us);
	printSerial("	SPL abs difference = %.2f\n",SPL - SPL_expected);

	for (uint32_t i = 0; i<SOUND_FREQ_BANDS; i++) {
		// convert the result to 1.d.p for a fair test:
		bandSPL[i] = roundf(bandSPL[i]*10.0)/10.0;
		printSerial("	Band %i abs difference = %.2f\n",i,bandSPL[i] - bandSPL_expected[i]);
	}
	printSerial("\n");

	////////////////////////////

	printSerial("calculateSPLfast() test\n");

	memcpy((int32_t *) &dataBuffer, &x1_32, 4*FFT_N); // 3rd arg is #bytes

	RESET_TMR15_AND_FLAG;
	calculateSPLfast();
	GET_TIME_TMR15(time_us);

	float theSPL = ((float) SPL_int) + (((float) SPL_frac_1dp)/10.0);

	printSerial("	Time taken for SPL calc is: %u us\n",time_us);
	printSerial("	SPL abs difference = %.2f\n",theSPL - SPL_expected);

	for (uint32_t i = 0; i<SOUND_FREQ_BANDS; i++) {
		theSPL = ((float) bandSPL_int[i]) + (((float) bandSPL_frac_1dp[i])/10.0);
		//printSerial("	Band %i SPL = %i.%i = %.1fF, expected: %.1f\n",
		//		i,bandSPL_int[i],bandSPL_frac_1dp[i],theSPL,bandSPL_expected[i]);
		printSerial("	Band %i abs difference = %.2f\n",i,theSPL - bandSPL_expected[i]);
	}
	printSerial("\n");

	/////////////////////////////////////////////////////////////////////////////

	// SPL values of 30 - 90 dB require log10() on numbers between 1.72922e9 -> 1.72922e15
	#define nlogs 5
	volatile float input[nlogs] = {1.72922e9, 3.7676e10, 9.0e12, 1.9283e14, 1.72922e15};
	volatile float exp_ans[nlogs] = {9.23785025, 10.57606, 12.9542425, 14.2851746, 15.23785025};
	for (uint32_t i=0; i<nlogs; i++) {
		RESET_TMR15_AND_FLAG;
		volatile float ans = log10(input[i]);
		GET_TIME_TMR15(time_us);
		pcDiff = (100.0*(ans - exp_ans[i]))/exp_ans[i];
		printSerial("log10() percentage difference = %.2f%%\n",pcDiff);
		printSerial("Time for log10(): %u us\n",time_us);
	}

	printSerial("\n");
	volatile float A = 3.141592e2;
	volatile float B = 29.762e10;
	volatile bool comp = false;
	RESET_TMR15_AND_FLAG;
	if (B > A) {
		comp = true;
	}
	GET_TIME_TMR15(time_us);
	if (!comp) {
		printSerial("Float compare FAILED\n");
	}
	else {
		printSerial("Time for a float compare: %u us\n",time_us);
	}

	printSerial("\n");
	volatile float C = 7.141592e2;
	volatile float exp_ACans = 1028.3184;
	RESET_TMR15_AND_FLAG;
	volatile float ACans = A+C;
	GET_TIME_TMR15(time_us);
	pcDiff = (100.0*(ACans - exp_ACans))/exp_ACans;
	printSerial("Float addition percentage difference = %.2f%%\n",pcDiff);
	printSerial("Time for a float addition: %u us\n",time_us);

	printSerial("\n");
	volatile float exp_AmCans = 224359.6829;
	RESET_TMR15_AND_FLAG;
	volatile float AmCans = A*C;
	GET_TIME_TMR15(time_us);
	pcDiff = (100.0*(AmCans - exp_AmCans))/exp_AmCans;
	printSerial("Float multiplication percentage difference = %.2f%%\n",pcDiff);
	printSerial("Time for a float multiplication: %u us\n",time_us);

	printSerial("\n");
	volatile float exp_CdAans = 2.27323981;
	RESET_TMR15_AND_FLAG;
	volatile float CdAans = C/A;
	GET_TIME_TMR15(time_us);
	pcDiff = (100.0*(CdAans - exp_CdAans))/exp_CdAans;
	printSerial("Float division percentage difference = %.2f%%\n",pcDiff);
	printSerial("Time for a float division: %u us\n",time_us);

	printSerial("\n");
	volatile uint64_t u64 = UINT64_C(0x40A0900F04004000);
	RESET_TMR15_AND_FLAG;
	volatile float u64f = (float) u64;
	GET_TIME_TMR15(time_us);
	u64f+=1.0; // just so it is used
	printSerial("Time for a u64->float cast: %u us\n",time_us);

	/////////////////////////////////////////////////////////////////////////////

	// efficient 10log10
	// NB: GCC compiler memory barrier is:   asm volatile("" ::: "memory");

	printSerial("\n");
	printSerial("efficient_10log10() tests\n");

	volatile int32_t integerPart, fractionalPart;

	for (uint32_t i=0; i<Ntest_inputs_10log10; i++) {
		RESET_TMR15_AND_FLAG;
		efficient_10log10(testInputs[i], (int32_t *) &integerPart, (int32_t *) &fractionalPart);
		GET_TIME_TMR15(time_us);
		printSerial("efficient_10log10() ran in = %u us\n",time_us);
		float result = ((float) integerPart) + (((float) fractionalPart)/10.0);
		float absErr = result - testTrueOutputs[i];
		printSerial("                    absErr = %.2f\n",absErr);

		RESET_TMR15_AND_FLAG;
		volFloat = 10.0*log10((float) testInputs[i]);
		GET_TIME_TMR15(time_us);
		printSerial("float version ran in = %u us\n",time_us);
		// convert the result to 1.d.p for a fair test:
		volFloat = roundf(volFloat*10.0)/10.0;
		printSerial("        float absErr = %.2f\n",volFloat - testTrueOutputs[i]);
	}

	printSerial("\n");

	///////////////////////////////////////////////////////

	printSerial("correctIntFracNumber() tests:\n");

	ok = true;
	integerPart = 0;
	fractionalPart = 2;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 0) && (fractionalPart == 2);

	integerPart = -7;
	fractionalPart = -5;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == -7) && (fractionalPart == -5);

	integerPart = 3;
	fractionalPart = 4;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 3) && (fractionalPart == 4);

	integerPart = 2;
	fractionalPart = 12;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 3) && (fractionalPart == 2);

	integerPart = -2;
	fractionalPart = -21;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == -4) && (fractionalPart == -1);

	integerPart = 2;
	fractionalPart = -6;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 1) && (fractionalPart == 4);

	integerPart = -10;
	fractionalPart = 22;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == -7) && (fractionalPart == -8);

	integerPart = 10;
	fractionalPart = -31;
	printSerial("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	printSerial("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 6) && (fractionalPart == 9);

	printSerial("correctIntFracNumber: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	printSerial("\n");

	///////////////////////////////////////////////////////

	return testsOK;
}

#endif

