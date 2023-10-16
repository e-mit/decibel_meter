#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include "sensor_constants.h"

#define SPLBANDS_INT_BYTES SOUND_FREQ_BANDS
#define SPLBANDS_FRAC_BYTES SOUND_FREQ_BANDS

extern volatile SoundData_t soundData;
extern volatile SoundData_t soundData_internal;

#endif

