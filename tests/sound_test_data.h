// Test data for system tests.

#ifndef SOUND_TEST_DATA_H
#define SOUND_TEST_DATA_H

#include <stdint.h>
#include <arm_math.h>

#define X0_LEN 2048
extern const float32_t testInput_f32_10khz[X0_LEN];
extern const float32_t FFT_testInput_f32_10khz[(X0_LEN/4)+1];
extern const float32_t real_FFT_testInput_f32_10khz[X0_LEN/4];

#define X1_32_LEN 1024
extern const int32_t x1_32[X1_32_LEN];
extern const float32_t FFT_x1_32[X1_32_LEN/2];
extern const float32_t x1_32_SPL_dBA;

#define X2_32_LEN 1024
extern const int32_t x2_32[X2_32_LEN];
extern const float32_t x2_32_SPL_dBA;

#endif
