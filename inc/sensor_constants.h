#ifndef SENSOR_CONSTANTS_H
#define SENSOR_CONSTANTS_H

#include <stdint.h>

// Frequency bands for sound level measurement
#define SOUND_FREQ_BANDS 6
static const uint16_t sound_band_mids_Hz[SOUND_FREQ_BANDS] = {125, 250, 500, 1000, 2000, 4000};
static const uint16_t sound_band_edges_Hz[SOUND_FREQ_BANDS+1] = {88, 177, 354, 707, 1414, 2828, 5657};

///////////////////////////////////////////////////////////

typedef struct __attribute__((packed)) {
  uint8_t  SPL_dBA_int;
  uint8_t  SPL_dBA_fr_1dp;
  uint8_t  SPL_bands_dB_int[SOUND_FREQ_BANDS];
  uint8_t  SPL_bands_dB_fr_1dp[SOUND_FREQ_BANDS];
  uint16_t peak_amp_mPa_int;
  uint8_t  peak_amp_mPa_fr_2dp;
  uint8_t  stable;
} SoundData_t;

#endif
