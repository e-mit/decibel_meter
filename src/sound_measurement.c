#include <sound_utilities.h>
#include <stdint.h>
#include <stdlib.h>
#include "arm_math.h"
#include "sound_measurement.h"
#include "arm_const_structs.h"
#include <stdbool.h>
#include "stm32g0xx_hal.h"
#include "sound_LUTs.h"
#include "math.h"

#define BIT_ROUNDING_MARGIN 4

/* Conversion of microphone digital output to sound pressure.
   This depends on the microphone sensitivity (S) and the
   output data bitdepth (N).
	  pressure/mPa = (digital amplitude)*ik_mPa;
	  ik_mPa = sqrt(2)/((10^((S/dB)/20))*((2^(N-1))-1))
   e.g. If S = -26 dB and N = 24, then: ik_mPa = 3.3638e-3
*/
const float ik_mPa = 3.3638e-3;
/* Decibel scale factor 'dBscale' is constant for a given microphone:
      dBscale = 20*log10((ik_mPa/1000)/(20e-6))
   e.g. if S = -26dB and N = 24; dBscale = -15.5 (to 1.d.p.)
*/
const int32_t dBscale_int = -15;
const int32_t dBscale_frac = -5;

// This function must be supplied externally:
extern void errorHandler(const char * func, uint32_t line, const char * file);

#ifdef DEBUG_PRINT
	// Debug data will be printed over the serial port
	extern void print(const char* format, ...);  // Supply this function externally
    extern void printU64hex(uint64_t x);  // Supply this function externally
    #define TEST_LENGTH_SAMPLES 20  // How many sound samples to print
#endif

////////////////////////////////////////////////

// state variables:
static volatile bool SPL_calc_enabled = false;
volatile bool DMAintEnabled = false; // whether interrupts can fire
static volatile bool SPL_calc_complete = false; // set true after every SPL calculation, which may be on
										        // every DMA ISR OR every N ISRs, depending on filter settings.

// variables as integer-fraction representation:
static volatile int32_t SPL_int = 0, SPL_frac_1dp = 0;
static volatile int32_t bandSPL_int[SOUND_FREQ_BANDS] = {0}, bandSPL_frac_1dp[SOUND_FREQ_BANDS] = {0};

static volatile uint32_t maximumAmplitude = 0; // stores the maximum until cleared by user
static volatile uint16_t dmaBuffer[FULL_BUFLEN] = {0}; // holds raw uint16s received from I2S module, used in 2 halves
static volatile int32_t dataBuffer[EIGHTH_BUFLEN] = {0}; // holds final 24-bit sound data from half of the DMA

// Handles for peripherals:
static TIM_HandleTypeDef * hSettleTimer;
static I2S_HandleTypeDef * hI2S;
static DMA_HandleTypeDef * hDMA;
static IRQn_Type DMA_Channel_IRQn;

// Counter used to ignore the maximum amplitude for the first
// N_AMP_SETTLE_PERIODS half-DMA interrupts, allowing the filter to settle.
static volatile uint32_t amplitudeSettlingPeriods = 0;

#ifdef FILTER_SPL
	static volatile int32_t spl_int_sum = 0;
	static volatile int32_t spl_frac1dp_sum = 0;
	static volatile int32_t band_spl_int_sum[SOUND_FREQ_BANDS] = {0};
	static volatile int32_t band_spl_frac1dp_sum[SOUND_FREQ_BANDS] = {0};
	static volatile uint32_t spl_sum_count = 0;
#endif

//////////////////////////////////////////////////////////////////////////////

static bool startMicSettlingPeriod(void);
static void decodeI2SdataLch(const uint16_t * inBuf, const uint32_t inBuflen, int32_t * outBuf);
static void processHalfDMAbuffer(uint32_t halfBufferStart);
static void calculateSPLQ31(void);
static uint32_t getFilteredMaxAmplitudeQ31(const int32_t * data, const uint32_t length,
										   bool reset, bool updateMaxAmpFollower);
static bool micSettlingComplete(void);
static void reset_SPL_state(void);

//////////////////////////////////////////////////////////////////////////////

volatile bool isSPLcalcComplete(void) {
	return SPL_calc_complete;
}

// Obtain the output SoundData, during a brief period of disabled DMA interrupt.
// Note that disabling the interrupt prevents the possibility of corrupted data
// but does not (under non-error conditions) cause loss of sound data because the
// DMA buffer is still being filled with I2S data.
void getSoundData(SoundData_t * data, bool getSPLdata, bool getMaxAmpData) {
	if (DMAintEnabled) {
		NVIC_DisableIRQ(DMA_Channel_IRQn);
	}
	// Use memory barrier instructions here, in case DMA interrupt had already been triggered
	// and would execute in the next few cycles. NB: __DMB is not needed.
	__DSB();
	__ISB();
	// At this point, we know that no DMA ISR is in progress and that it will not trigger until re-enabled.

	if (getSPLdata) {

	#ifdef FILTER_SPL
		if (spl_sum_count == 0) {
			// No data: prevent divide by zero
			data->SPL_dBA_int = 0;
			data->SPL_dBA_fr_1dp = 0;
			for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
				data->SPL_bands_dB_int[i] = 0;
				data->SPL_bands_dB_fr_1dp[i] = 0;
			}
		}
		else {
			sumToIntAverage(&(data->SPL_dBA_int), &(data->SPL_dBA_fr_1dp), spl_int_sum,
					           spl_frac1dp_sum, spl_sum_count);

			for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
				sumToIntAverage(&(data->SPL_bands_dB_int[i]), &(data->SPL_bands_dB_fr_1dp[i]),
						           band_spl_int_sum[i],	band_spl_frac1dp_sum[i], spl_sum_count);
			}
		}
	#else
		if (SPL_int < 0) {
			data->SPL_dBA_int = 0;
			data->SPL_dBA_fr_1dp = 0;
		}
		else if (SPL_int > UINT8_MAX) {
			data->SPL_dBA_int = UINT8_MAX;
			data->SPL_dBA_fr_1dp = 9;
		}
		else {
			data->SPL_dBA_int = (uint8_t) SPL_int;
			data->SPL_dBA_fr_1dp = (uint8_t) SPL_frac_1dp;
		}

		for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
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

	if (getMaxAmpData) {
		uint16_t intPart = 0;
		uint8_t fracPart = 0;
		amplitude_DN_to_mPa(maximumAmplitude, ik_mPa, &intPart, &fracPart);
		data->peak_amp_mPa_int = intPart;
		data->peak_amp_mPa_fr_2dp = fracPart;
	}

	data->stable = micSettlingComplete();

	if (DMAintEnabled) {
		NVIC_EnableIRQ(DMA_Channel_IRQn);
	}
	// NOTE that any pending DMA interrupt will now fire, but will take ~2 cycles to start
}

void DMA1_Channel1_IRQHandler(void) {
	HAL_DMA_IRQHandler(hDMA);
}

// Initialize hardware for reading out the microphone: DMA, Timer, I2S.
// Obtain the handles to the peripherals and return bool success.
bool soundInit(void (*DMAInit)(DMA_HandleTypeDef **), bool (*I2SInit)(I2S_HandleTypeDef **),
		       bool (*tmrInit)(TIM_HandleTypeDef **), IRQn_Type DMAChIRQn) {
	(*DMAInit)(&hDMA);
	DMA_Channel_IRQn = DMAChIRQn;
	bool ok = (*I2SInit)(&hI2S);
	ok = ok && (*tmrInit)(&hSettleTimer);
	return ok;
}

// Call this from external code to clear the maximum amplitude value.
void clearMaximumAmplitude(void) {
	maximumAmplitude = 0;
}

// Prepare a one-shot timer to indicate the short time period during which the
// microphone output is inaccurate after power-on (warmup/settling time).
// This is output with the data for advice only: all functions still operate as
// normal during this period.
static bool startMicSettlingPeriod(void) {
	__HAL_TIM_SetCounter(hSettleTimer, 0);
	if (HAL_TIM_Base_Start(hSettleTimer) != HAL_OK) {
		return false;
	}
	return true;
}

// See whether the warmup/settling time has finished
static bool micSettlingComplete(void) {
	bool complete = __HAL_TIM_GET_FLAG(hSettleTimer, TIM_SR_UIF);
	if (complete) {
		HAL_TIM_Base_Stop(hSettleTimer);
	}
	return complete;
}

// Enable: starts the I2S clock, warmup timer, and DMA interrupts
// Disable: stops the DMA interrupts and stops I2S clock.
bool enableMicrophone(bool bEnable) {
	static bool micEnabled = false;
	if (bEnable == micEnabled) {
		return true;
	}
	if (bEnable) {
		startMicSettlingPeriod();
		if (HAL_I2S_Receive_DMA(hI2S, (uint16_t *) dmaBuffer, HALF_BUFLEN) != HAL_OK) {
			return false;
		}
		clearMaximumAmplitude();
		amplitudeSettlingPeriods = 0;
		NVIC_EnableIRQ(DMA_Channel_IRQn);
		DMAintEnabled = true;
		micEnabled = true;
	}
	else {
		NVIC_DisableIRQ(DMA_Channel_IRQn);
		DMAintEnabled = false;
		enableSPLcalculation(false);
		if (HAL_I2S_DMAStop(hI2S) != HAL_OK) {
			return false;
		}
		micEnabled = false;
	}
	return true;
}

// Convert input raw I2S data into signed 32 bit numbers, assuming the I2S data is Left
// channel only and the first datum starts at element 0.
// inBuflen is simply the number of elements in inBuf
static void decodeI2SdataLch(const uint16_t * inBuf, const uint32_t inBuflen, int32_t * outBuf) {
	uint32_t outCount = 0;
	for (uint32_t i = 0; i < inBuflen; i += 4) {
		// join MS16bits and LS16bits, then shift the result down 8 bits because it is a 24-bit
		// value, rather than a 32-bit one.
		outBuf[outCount] = ((int32_t) ((((uint32_t) inBuf[i]) << 16) | ((uint32_t) inBuf[i+1]))) >> 8;
		outCount++;
	}
}

void enableSPLcalculation(bool bEnable) {
	if (bEnable) {
		reset_SPL_state();
	}
	SPL_calc_enabled = bEnable;
}

// Called from the DMA ISR when the first half of the DMA buffer is full,
// i.e. "HALF_BUFLEN" uint16s are in the first half of dmaBuffer
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	processHalfDMAbuffer(0);
}

// Called from the DMA ISR when the second half of the DMA buffer is full,
// i.e. "HALF_BUFLEN" uint16s are in the second half of dmaBuffer
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	processHalfDMAbuffer(HALF_BUFLEN);
}

static void processHalfDMAbuffer(uint32_t halfBufferStart) {
	// Decode the raw I2S data and copy it out of the DMA buffer and into dataBuffer
	decodeI2SdataLch((uint16_t *) &(dmaBuffer[halfBufferStart]), HALF_BUFLEN, (int32_t *) dataBuffer);
	// Filter the amplitude, find the maximum, and update maximumAmplitude if necessary:
	getFilteredMaxAmplitudeQ31((int32_t *) dataBuffer, (uint32_t) EIGHTH_BUFLEN,
			                   amplitudeSettlingPeriods == 0,
							   amplitudeSettlingPeriods >= N_AMP_SETTLE_HALF_PERIODS);
	if (amplitudeSettlingPeriods < N_AMP_SETTLE_HALF_PERIODS) {
		// Need to allow the IIR filter to settle
		amplitudeSettlingPeriods++;
	}
	if (SPL_calc_enabled) {
		// Calculate the A-weighted SPL and octave bands SPL
		calculateSPLQ31();
	}
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
	errorHandler(__func__, __LINE__, __FILE__);
}

// Calculate A-weighted SPL and frequency-band SPL from input data.
// dataBuffer must contain (at least) FFT_N values
// dataBuffer is reused for storage throughout this function. Since dataBuffer is shared by both halves of
// the DMA buffer, this function must complete before the next DMA interrupt.
// This uses Q31 integers, rather than floats, for improved speed.
static void calculateSPLQ31(void) {
	static q31_t FFTdata[2*FFT_N] = {0}; // interleaved complex, so need 2x number of elements.

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
			#warning("This choice of FFT_N and I2S_FREQ results in NO signal in the lowest octave band")
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

	// Find max, min values of the input data
	int32_t max, min;
	findMinMax(&min, &max, (int32_t *) dataBuffer, FFT_N);

	// Calculate the centre of the signal range, and the largest bitshift needed to
	// fill the available range without saturating
	int32_t centre = (min/2) + (max/2);
	uint32_t amplitude = (uint32_t) (max - centre + BIT_ROUNDING_MARGIN);
	uint32_t bitShift = getPo2factor(INT32_MAX, amplitude);

	// Apply offset and bitshift and put data into FFT input array
	uint32_t count = 0;
	for (uint32_t i = 0; i < FFT_N; i++) {
		FFTdata[count] = (q31_t) ((dataBuffer[i] - centre) << bitShift);
		FFTdata[count + 1] = 0;
		count += 2;
	}
	#ifdef DEBUG_PRINT
		print("Input: min = %i, max = %i, centre = %i, amplitude = %u, bitshift = %u\n\n",
				     min, max, centre, amplitude, bitShift);
		print("Scaled FFT real input:\n");
		for (uint32_t i = 0; i < (2*TEST_LENGTH_SAMPLES); i += 2) {
			print("%i\n", FFTdata[i]);
		}
		print("\n");
	#endif

	// Calculate the FFT; the output is internally divided by FFT_N (number of points)
	arm_cfft_q31(&S, FFTdata, 0, 1);

	#ifdef DEBUG_PRINT
		print("Raw FFT output:\n");
		for (uint32_t i = 0; i < (TEST_LENGTH_SAMPLES); i++) {
			print("%i\n", FFTdata[i]);
		}
		print("\n");
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
	// Calculate the largest bitshift needed to fill the available range without saturating
	uint32_t amplitude2 = ((uint32_t) max) + BIT_ROUNDING_MARGIN;
	uint32_t bitShift2 = getPo2factor((uint32_t) INT32_MAX, amplitude2);

	// Apply the bitshift (not to the dc bins, and to 1st half of data only), then get the
	// absolute square magnitude of each bin
	for (uint32_t i = 2; i<FFT_N; i++) {
		FFTdata[i] = (q31_t) (FFTdata[i] << bitShift2);
	}
	q31_t * sqmag = (q31_t *) dataBuffer; // this re-uses the input dataBuffer as working memory
	arm_cmplx_mag_squared_q31(FFTdata, sqmag, FFT_N/2);

	// Apply the A-weighting and sum, excluding the dc bin.
	// Also sum for the unweighted frequency-band SPL:
	uint64_t bandSum[SOUND_FREQ_BANDS] = {0};
	uint64_t sumSq = 0; // sum of squared weighted magnitudes (scaled)
	for (uint32_t i = 1; i < (FFT_N/2); i++) {
		sumSq += ((uint64_t) sqmag[i])*((uint64_t) sqWsc[i]);
		if (bandIDs[i] != SOUND_FREQ_BANDS) {
			// This indicates that the FFT bin does belong in one of the frequency-bands:
			bandSum[bandIDs[i]] += (uint64_t) sqmag[i];
		}
	}

	// Reverse the (explicit and implicit) scalings using a bitshift.
	// Shifts applied before squaring are doubled when reversed.
	int32_t bs_right = ((int32_t) (2*bitShift)) + ((int32_t) (2*bitShift2)) -3 -31;
	uint32_t absShift = (uint32_t) abs(bs_right);
	if (bs_right < 0) {
		// Left shift
		sumSq = sumSq << absShift;
		for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
			bandSum[i] = bandSum[i] << absShift;
		}
	}
	else {
		// Right shift
		sumSq = sumSq >> absShift;
		for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
			bandSum[i] = bandSum[i] >> absShift;
		}
	}

	// Add on the dB terms accounting for the microphone parameters
	// and (only for the A-weighted SPL) the weighting scale factor
	scaleSPL(sumSq, dBscale_int, dBscale_frac, tenlog10SF_int, tenlog10SF_frac,
			 (int32_t *) &SPL_int, (int32_t *) &SPL_frac_1dp);
	for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
		scaleSPL(bandSum[i], dBscale_int, dBscale_frac, 0, 0,
				 (int32_t *) &(bandSPL_int[i]), (int32_t *) &(bandSPL_frac_1dp[i]));
	}

	#ifdef FILTER_SPL
		spl_int_sum += SPL_int;
		spl_frac1dp_sum += SPL_frac_1dp;

		for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
			band_spl_int_sum[i] += bandSPL_int[i];
			band_spl_frac1dp_sum[i] += bandSPL_frac_1dp[i];
		}

		spl_sum_count++;
		if (spl_sum_count >= FILTER_SPL_N) {
			SPL_calc_complete = true;
			SPL_calc_enabled = false;
		}
	#else
		SPL_calc_complete = true;
	#endif

	#ifdef DEBUG_PRINT
		print("FFT output: max = %i, amplitude = %u, bitshift = %u\n\n", max, amplitude2, bitShift2);

		print("Scaled FFT output:\n");
		for (uint32_t i = 0; i < TEST_LENGTH_SAMPLES; i++) {
			print("%i\n", FFTdata[i]);
		}
		print("\n");

		print("Squared magnitude:\n");
		for (uint32_t i = 0; i < (TEST_LENGTH_SAMPLES/2); i++) {
			print("%i\n", sqmag[i]);
		}
		print("\n");

		print("sumSq = ");
		printU64hex(sumSq);
		print("\n");
		for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
			print("Band %i sumSq = ",i);
			printU64hex(bandSum[i]);
			print("\n");
		}
		print("bs_right = %i\n\n", bs_right);
	#endif
}

static void reset_SPL_state(void) {
	#ifdef FILTER_SPL
		spl_int_sum = 0;
		spl_frac1dp_sum = 0;
		spl_sum_count = 0;
		for (uint32_t i=0; i<SOUND_FREQ_BANDS; i++) {
			band_spl_int_sum[i] = 0;
			band_spl_frac1dp_sum[i] = 0;
		}
	#endif
	SPL_calc_complete = false;
}

// Find and return the largest absolute amplitude in the input data buffer.
// Optionally also update the global maximum amplitude value if the new maximum is larger.
// Uses a simple single-pole hi-pass IIR filter to remove the dc offset in the input data.
// Uses Q31 operations.
static uint32_t getFilteredMaxAmplitudeQ31(const int32_t * data, const uint32_t length,
										   bool reset, bool updateMaxAmpFollower) {
	static q31_t filtered = 0;
	static q31_t lastData = 0;
	// Want a filter with cutoff of fc = 10Hz. The coefficients depend on Fs according to:
	// b = exp(-2.pi.(1/Fs).fc)
	// a0 = (1+b)/2
	// a1 = -a0
	// Then convert to Q31 by dividing by 2^-31 and then rounding.
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
		// Reset the state of the digital filter; e.g. if the mic has been disabled then re-enabled.
		filtered = 0;
		lastData = 0;
	}
	q31_t maxAmp = 0;
	q31_t minAmp = 0;

	// Apply a bitshift to the incoming data, before filtering, to maximise the dynamic range
	// BUT while also ensuring the intermediate value cannot overflow (three Q31 values are added together).
	const uint32_t scalingBitShift = 5;
	for (uint32_t i = 0; i < length; i++) {
		q31_t fx = (q31_t) (data[i] << scalingBitShift);

		// Arm saturating operations:
		// D = A*B is: arm_mult_q31(&A, &B, &D, 1);
		// D = A+B is: arm_add_q31(&A, &B, &D, 1);

		// Now do the filter calculation:
		// filtered = (a0*fx) + (a1*lastData) + (b*filtered);
		// BUT NOTE that since a1 = -a0 this becomes:
		// filtered = (a0*(fx - lastData)) + (b*filtered)
		q31_t r1, r2, r3;
		lastData = -lastData;
		arm_add_q31(&fx, &lastData, &r1, 1); // r1 = fx - lastData
		arm_mult_q31(&a0, &r1, &r2, 1);      // r2 = a0*r1
		arm_mult_q31(&b, &filtered, &r3, 1); // r3 = b*filtered
		arm_add_q31(&r2, &r3, &filtered, 1); // filtered = r2 + r3

		lastData = fx;
		if (filtered > maxAmp) {
			maxAmp = filtered;
		}
		else if (filtered < minAmp) {
			minAmp = filtered;
		}
	}
	// Find the maximum absolute amplitude from the signed values:
	uint32_t absMin = abs(minAmp);
	uint32_t absMax = (uint32_t) maxAmp;
	uint32_t absMaxAmp = (absMin > absMax) ? absMin : absMax;

	// Reverse the scaling bitshift
	uint32_t absMaxAmp32 = (uint32_t) (absMaxAmp >> scalingBitShift);

	if (updateMaxAmpFollower && (absMaxAmp32 > maximumAmplitude)) {
		maximumAmplitude = absMaxAmp32;
	}
	return absMaxAmp32;
}


