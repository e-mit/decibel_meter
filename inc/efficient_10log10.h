#ifndef EFFICIENT_10LOG10_LUTS_H
#define EFFICIENT_10LOG10_LUTS_H

#include <stdint.h>

// these LUTs were chosen with the matlab program

extern const uint32_t mantissa_kBits;

#define LUT_10LOG10_LENGTH 128
extern const uint8_t LUT10log10[];
extern const uint32_t intOffset10log10;
extern const uint32_t fracPartLSBs;

extern const uint16_t TLT2_intPart;
extern const uint16_t TLT2_shiftedFracPart;
extern const uint32_t TLT2_bitshift;

//////////////////////////////
// test data for verifying the 10log10 function:
#define Ntest_inputs_10log10 20
extern const uint64_t testInputs[];
// these are the expected values of 10*log10(testInputs), given to 1.d.p
extern const float testTrueOutputs[];
/////////////////////////////////////////

void efficient_10log10(uint64_t P, int32_t * integerPart, int32_t * fractionalPart);
void correctIntFracNumber(int32_t * intPart, int32_t * fracPart);

#endif
