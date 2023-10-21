#include <math.h>
#include <sound_utilities.h>
#include <stdint.h>
#include "efficient_10log10.h"

// assume float input is positive and can fit in a uint32
// to print, do: printf("%i.%02i\n",intpart, fracpart2dp);
// to convert to float, do intpart + (fracpart/100)
void float2IntFrac2dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart2dp) {
	uint32_t rounded = (uint32_t) roundf(positiveValue*100.0);
	intpart[0] = rounded/100;
	fracpart2dp[0] = (uint8_t) (rounded - (intpart[0]*100));
}

// assume float input is positive and can fit in a uint32
// to print, do: printf("%i.%i\n",intpart, fracpart1dp);
// to convert to float, do intpart + (fracpart/10)
void float2IntFrac1dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart1dp) {
	uint32_t rounded = (uint32_t) roundf(positiveValue*10.0);
	intpart[0] = rounded/10;
	fracpart1dp[0] = (uint8_t) (rounded - (intpart[0]*10));
}

// Convert an accumulated SPL sum into an average value, in (integer, fractional) format.
void SPLsumToIntAverage(uint8_t * SPL_integer, uint8_t * SPL_fractional_1dp, const int32_t splIntSum,
		                const int32_t splFrac1dpSum, const uint32_t sumCount) {
	float splAverage = (((float) splIntSum) +
					   (((float) splFrac1dpSum)/10.0))/((float) sumCount);
	uint32_t intpart = 0;
	uint8_t fracpart1dp = 0;
	float2IntFrac1dp(splAverage, &intpart, &fracpart1dp);

	if (intpart > UINT8_MAX) {
		// Far beyond the range of the sensor component
		SPL_integer[0] = UINT8_MAX;
		SPL_fractional_1dp[0] = 9;
	}
	else {
		SPL_integer[0] = (uint8_t) intpart;
		SPL_fractional_1dp[0] = (uint8_t) fracpart1dp;
	}
}


// Find smallest and largest ints in array
void findMinMax(int32_t * min, int32_t * max, const int32_t * array, const uint32_t length) {
	max[0] = INT32_MIN;
	min[0] = INT32_MAX;
	for (uint32_t i = 0; i < length; i++) {
		if (array[i] < min[0]) {
			min[0] = array[i];
		}
		if (array[i] > max[0]) {
			max[0] = array[i];
		}
	}
}

// Find the largest positive integer bitshift m, such that: smallVal*(2^m) <= bigVal.
// This is the largest upward bitshift that can be applied to smallVal such
// that it is still smaller than bigVal.
uint32_t getPo2factor(uint32_t bigVal, uint32_t smallVal) {
	uint32_t bitShift = 0;
	if ((bigVal < smallVal) || (smallVal == 0)) {
		return 0;
	}
	while (bigVal >= smallVal) {
		bigVal = bigVal >> 1;
		bitShift++;
	}
	bitShift -= 1; // correction for going one shift too far
	return bitShift;
}

// Convert an amplitude value in "DN" (digital numbers) to amplitude in milliPascals.
// This works for peak amplitude, general amplitude, RMS amplitude.
// Returns integer and fractional part to 2 d.p.
// Given that ampDN is at most 2^24, the output will always fit in a uint16.
void amplitude_DN_to_mPa(const uint32_t ampDN, uint16_t * intAmp_mPa, uint8_t * frac2dpAmp_mPa) {
	/* NOTE: depends on the microphone sensitivity (S) and data bitdepth (N)
	amp/Pa = (amp/DN)*ik;
	where ik = sqrt(2)/((10^((S/dB)/20))*((2^(N-1))-1))
	e.g. If S = -26 dB and N = 24, then: ik = 3.3638e-3 mPa/DN
	*/
	const float ik_mPa = 3.3638e-3;
	float amp = ((float) ampDN)*ik_mPa;
	uint32_t intpart = 0;
	float2IntFrac2dp(amp, &intpart, frac2dpAmp_mPa);
	intAmp_mPa[0] = (uint16_t) intpart;
}

// Find the final SPL value by adding the terms accounting for the
// microphone parameters and (only for weighted SPL) the weighting scale factor
void scaleSPL(uint64_t sumSq, const int32_t add_int, const int32_t add_frac,
		      int32_t * SPLintegerPart, int32_t * SPLfractionalPart) {
	/* micTerm is constant for a given microphone:
	const = sqrt(2)/(((2^(Nbits-1))-1)*(10^((S/dB)/20))); % = 3.3638e-06
	micTerm = 20*log10(const/(20e-6)); % = -1.5484097e+01 = -15.5 to 1.d.p.
	These values assume the mic sensitivity S = -26dB
	 */
	const int32_t micTerm_int = -15;
	const int32_t micTerm_frac = -5;

	// Calculate: SPLvalue = (10.0*log10(sumSq)) + micTerm + addTerm;
	efficient_10log10(sumSq, SPLintegerPart, SPLfractionalPart);
	SPLintegerPart[0] = SPLintegerPart[0] + micTerm_int + add_int;
	SPLfractionalPart[0] = SPLfractionalPart[0] + micTerm_frac + add_frac;
	// Apply correction if fractional part is not in range 0->9:
	correctIntFracNumber(SPLintegerPart, SPLfractionalPart);
}
