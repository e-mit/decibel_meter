#include "efficient_10log10.h"

// the LUTs etc are at the bottom of this file

// get the entry of the LUT and extract the two parts:
static void accessLUT10log10(uint32_t ind, uint32_t * intPart, uint32_t * fracPart) {

	if (ind >= LUT_10LOG10_LENGTH) {
		ind = LUT_10LOG10_LENGTH - 1;
	}
	uint8_t x = LUT10log10[ind];

	intPart[0] = (uint32_t) (x >> fracPartLSBs);
	fracPart[0] = ((uint32_t) x) - (intPart[0] << fracPartLSBs);
}

// calculate 10*log10(P), returning the result as an integer part and a 1dp fractional part (implicit /10 needed)
void efficient_10log10(uint64_t P, int32_t * integerPart, int32_t * fractionalPart) {

	// NOTE: the CLZ (count leading zeros) instruction is not available on M0/M0+
	// so use simple shifting algorithm:

	uint64_t limit = (((uint64_t) 1) << mantissa_kBits) - 1; // limit = ((2^kBits)-1)

	// bit-shift P downwards until the remaining mantissa is a "mantissa_kBits" length value
	uint16_t bShift = 0;
	while (P > limit) {
		P = P >> 1;
		bShift++;
	}

	// calculate the index to the LUT:
	uint32_t ind = ((uint32_t) P) - (((uint32_t) 1) << (mantissa_kBits-1));  // subtracting 2^(kBits-1)

	// do the lookup:
	uint32_t lutIntPart = 0;
	uint32_t lutFracPart = 0;
	accessLUT10log10(ind, &lutIntPart, &lutFracPart);

	// provide the result as an integer and fractional part (with the fractional part x10)
	int32_t intPart = (int32_t) (intOffset10log10 + lutIntPart + (bShift*TLT2_intPart));
	int32_t fracPart = (int32_t) (lutFracPart + ((bShift*TLT2_shiftedFracPart) >> TLT2_bitshift));

	// Make a correction if fractional part is not in range 0->9:
	correctIntFracNumber(&intPart, &fracPart);

	// NB: result = integerPart + (fractionalPart/10);
	integerPart[0] = intPart;
	fractionalPart[0] = fracPart;
}

// after doing a sum of (+ve or -ve) numbers represented as integer and fractional part, correct
// it if the fractional part has grown so that the int part needs adjusting.
// NB: valid numbers have same sign of both int and frac parts (or one or either part zero) AND |fracPart| < 10
void correctIntFracNumber(int32_t * intPart, int32_t * fracPart) {
	while (fracPart[0] >= 10) {
		intPart[0] = intPart[0] + 1;
		fracPart[0] = fracPart[0] - 10;
	}
	while (fracPart[0] <= -10) {
		intPart[0] = intPart[0] - 1;
		fracPart[0] = fracPart[0] + 10;
	}
	// now correct cases where signs are not consistent:
	if ((intPart[0] < 0) && (fracPart[0] > 0)) {
		intPart[0] = intPart[0] + 1;
		fracPart[0] = fracPart[0] - 10;
	}
	else if ((intPart[0] > 0) && (fracPart[0] < 0)) {
		intPart[0] = intPart[0] - 1;
		fracPart[0] = fracPart[0] + 10;
	}
}

// LUTs (constants found using matlab program)
const uint32_t mantissa_kBits = 8; // must be less than 32

const uint16_t TLT2_intPart = 3;
const uint16_t TLT2_shiftedFracPart = 1;
const uint32_t TLT2_bitshift = 3;

const uint32_t intOffset10log10 = 21;
const uint32_t fracPartLSBs = 6;
const uint8_t LUT10log10[LUT_10LOG10_LENGTH] = {
1,
1,
2,
2,
2,
3,
3,
3,
4,
4,
4,
4,
5,
5,
5,
6,
6,
6,
7,
7,
7,
7,
8,
8,
8,
9,
9,
9,
9,
10,
64,
64,
65,
65,
65,
65,
66,
66,
66,
66,
67,
67,
67,
67,
68,
68,
68,
68,
69,
69,
69,
69,
70,
70,
70,
70,
71,
71,
71,
71,
72,
72,
72,
72,
72,
73,
73,
73,
73,
74,
74,
74,
128,
128,
129,
129,
129,
129,
129,
130,
130,
130,
130,
131,
131,
131,
131,
131,
132,
132,
132,
132,
132,
133,
133,
133,
133,
133,
134,
134,
134,
134,
134,
134,
135,
135,
135,
135,
135,
136,
136,
136,
136,
136,
136,
137,
137,
137,
137,
137,
138,
138,
138,
192,
192,
192,
193,
193};

const uint64_t testInputs[Ntest_inputs_10log10] = {
11170,
51992,
242002,
1126424,
5243060,
24404378,
113592759,
528729520,
2461027526,
11455113150,
53319036834,
248179101477,
1155175900896,
5376888521506,
25027296838757,
116492202609360,
542225289299535,
2523845010827701,
11747503785573328,
54679999999999952};

const float testTrueOutputs[Ntest_inputs_10log10] = {
40.5,
47.2,
53.8,
60.5,
67.2,
73.9,
80.6,
87.2,
93.9,
100.6,
107.3,
113.9,
120.6,
127.3,
134.0,
140.7,
147.3,
154.0,
160.7,
167.4};


