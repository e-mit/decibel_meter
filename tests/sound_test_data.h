// Test data for system tests.

#ifndef SOUND_TEST_DATA_H
#define SOUND_TEST_DATA_H

#include <stdint.h>

#define X0_LEN 2048
extern const float testInput_f32_10khz[X0_LEN];
extern const float FFT_testInput_f32_10khz[(X0_LEN/4)+1];
extern const float real_FFT_testInput_f32_10khz[X0_LEN/4];

#define X1_32_LEN 1024
extern const int32_t x1_32[X1_32_LEN];
extern const float FFT_x1_32[X1_32_LEN/2];
extern const float x1_32_SPL_dBA;

#define X2_32_LEN 1024
extern const int32_t x2_32[X2_32_LEN];
extern const float x2_32_SPL_dBA;

#endif
