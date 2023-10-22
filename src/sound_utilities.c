// Calculation functions used for sound measurements.
// These are not specific to the microphone hardware or the CPU architecture.

#include <math.h>
#include <sound_utilities.h>
#include <stdint.h>
#include "efficient_10log10.h"

// Convert a float to integer-fractional representation, where the
// fractional part has 2 decimal places.
// The float input must be positive and fit in a uint32.
// e.g. to print the result: printf("%i.%02i\n", intpart, fracpart2dp);
// To convert to float: intpart + (fracpart2dp/100.0)
void float2IntFrac2dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart2dp) {
	uint32_t rounded = (uint32_t) roundf(positiveValue*100.0);
	intpart[0] = rounded/100;
	fracpart2dp[0] = (uint8_t) (rounded - (intpart[0]*100));
}

// Convert a float to integer-fractional representation, where the
// fractional part has 1 decimal place.
// The float input must be positive and fit in a uint32.
// e.g. to print the result: printf("%i.%i\n", intpart, fracpart1dp);
// To convert to float: intpart + (fracpart1dp/10.0)
void float2IntFrac1dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart1dp) {
	uint32_t rounded = (uint32_t) roundf(positiveValue*10.0);
	intpart[0] = rounded/10;
	fracpart1dp[0] = (uint8_t) (rounded - (intpart[0]*10));
}

// Convert an accumulated sum into an average value, in (integer, fractional) format.
void sumToIntAverage(uint8_t * intpart, uint8_t * fracpart1dp, const int32_t intSum,
		             const int32_t frac1dpSum, const uint32_t sumCount) {
	float splAverage = (((float) intSum) +
					   (((float) frac1dpSum)/10.0))/((float) sumCount);
	uint32_t intpart32 = 0;
	float2IntFrac1dp(splAverage, &intpart32, fracpart1dp);

	if (intpart32 > UINT8_MAX) {
		intpart[0] = UINT8_MAX;
		fracpart1dp[0] = 9;
	}
	else {
		intpart[0] = (uint8_t) intpart32;
	}
}


// Find the smallest and largest integers in an array.
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
	bitShift -= 1; // do not count the final shift
	return bitShift;
}

// Convert an amplitude value in "DN" (digital numbers) to amplitude in milliPascals.
// The microphone scale factor is ik_mPa. Returns integer and fractional part to 2 d.p.
// Given that ampDN is at most 2^24, the output will always fit in a uint16.
void amplitude_DN_to_mPa(const uint32_t ampDN, const float ik_mPa, uint16_t * intAmp_mPa,
		                 uint8_t * frac2dpAmp_mPa) {
	float amp = ((float) ampDN)*ik_mPa;
	uint32_t intpart = 0;
	float2IntFrac2dp(amp, &intpart, frac2dpAmp_mPa);
	intAmp_mPa[0] = (uint16_t) intpart;
}

// Find the final SPL value in decibels by taking log, adding the terms accounting
// for the microphone parameters and (only for weighted SPL) the weighting scale factor.
void scaleSPL(uint64_t sumSq, const int32_t dBscale_int, const int32_t dBscale_frac,
		      const int32_t weightingInt, const int32_t weightingFrac,
		      int32_t * SPLintegerPart, int32_t * SPLfractionalPart) {
	// Calculate: SPLvalue = (10.0*log10(sumSq)) + dBscale + weightTerm;
	efficient_10log10(sumSq, SPLintegerPart, SPLfractionalPart);
	SPLintegerPart[0] = SPLintegerPart[0] + dBscale_int + weightingInt;
	SPLfractionalPart[0] = SPLfractionalPart[0] + dBscale_frac + weightingFrac;
	// Apply correction if fractional part is not in range 0->9:
	correctIntFracNumber(SPLintegerPart, SPLfractionalPart);
}