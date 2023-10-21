#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>
#include <stdbool.h>

// this type defines a delay in terms of the start time and duration
typedef struct specdel_t {
	bool inProgress;
	uint32_t startTime_tick; // this is a global HAL tick value
	uint32_t duration_ms;
} SpecifiedDelay_t;

void float2IntFrac2dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart2dp);
void float2IntFrac1dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart1dp);
void getStartupReason(void);
bool delayHasExpired(SpecifiedDelay_t * theDelay, uint32_t * timeLeft_ms);
uint32_t getRemainingDelay_ms(SpecifiedDelay_t * theDelay);
void clearSpecifiedDelay(SpecifiedDelay_t * theDelay);

#endif
