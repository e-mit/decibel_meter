#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>
#include <stdbool.h>

void float2IntFrac2dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart2dp);
void float2IntFrac1dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart1dp);
const char * getStartupReason(void);

#endif
