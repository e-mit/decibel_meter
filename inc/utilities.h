#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>
#include <stdbool.h>

void float2IntFrac2dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart2dp);
void float2IntFrac1dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart1dp);
void SPLsumToIntAverage(uint8_t * SPL_integer, uint8_t * SPL_fractional_1dp, const int32_t splIntSum,
		                const int32_t splFrac1dpSum, const uint32_t sumCount);
void findMinMax(int32_t * min, int32_t * max, const int32_t * array, const uint32_t length);
uint32_t getPo2factor(uint32_t bigVal, uint32_t smallVal);
void amplitude_DN_to_mPa(uint32_t ampDN, uint16_t * intAmp_mPa, uint8_t * frac2dpAmp_mPa);
void scaleSPL(uint64_t sumSq, const int32_t add_int, const int32_t add_frac,
		             int32_t * SPLintegerPart, int32_t * SPLfractionalPart);

#endif
