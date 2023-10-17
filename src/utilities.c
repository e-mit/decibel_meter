#include "utilities.h"
#include "stm32g0xx_hal.h"
#include <math.h>
#include <stdint.h>

// assume float input is positive and can fit in a uint32
// to print, do: printf("%i.%02i\n",intpart, fracpart2dp);
// to convert to float, do intpart + (fracpart/100)
void float2IntFrac2dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart2dp) {
	uint32_t rounded = (uint32_t) roundf(positiveValue*100.0);
	intpart[0] = rounded/100;
	fracpart2dp[0] = (uint8_t) (rounded - (intpart[0]*100));
}

// assume float input is positive and can fit in a uint32
// to print, do: printf("%i.%i\n",intpart, fracpart1dp);
// to convert to float, do intpart + (fracpart/10)
void float2IntFrac1dp(float positiveValue, uint32_t *intpart, uint8_t *fracpart1dp) {
	uint32_t rounded = (uint32_t) roundf(positiveValue*10.0);
	intpart[0] = rounded/10;
	fracpart1dp[0] = (uint8_t) (rounded - (intpart[0]*10));
}


extern void printSerial(const char* format, ...);

const char * getStartupReason(void) {
	const char * reason = "Unknown";
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST) == 1) {
		reason = "BOR or POR/PDR reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) == 1) {
		reason = "Option byte loader reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == 1) {
		reason = "NRST pin reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == 1) {
		reason = "Software reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == 1) {
		reason = "Independent Watchdog reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) == 1) {
		reason = "Window watchdog reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == 1) {
		reason = "Low power reset";
	}
	/* Clear source Reset Flag */
	__HAL_RCC_CLEAR_RESET_FLAGS();
	return reason;
}

// works for delays up to 2^32 ms (50 days) and handles up to one rollover correctly
bool delayHasExpired(SpecifiedDelay_t * theDelay, uint32_t * timeLeft_ms) {
	uint32_t time_elapsed_ms = HAL_GetTick() - theDelay->startTime_tick;
	bool expired = (time_elapsed_ms >= theDelay->duration_ms);
	if (expired) {
		timeLeft_ms[0] = 0;
		return true;
	}
	else {
		timeLeft_ms[0] = theDelay->duration_ms - time_elapsed_ms;
		return false;
	}
}

// returns zero if delay has expired
uint32_t getRemainingDelay_ms(SpecifiedDelay_t * theDelay) {
	if (theDelay->inProgress) {
		uint32_t time_elapsed_ms = HAL_GetTick() - theDelay->startTime_tick;
		if (time_elapsed_ms >= theDelay->duration_ms) {
			// expired:
			theDelay->inProgress = false;
			return 0;
		}
		else {
			return (theDelay[0].duration_ms - time_elapsed_ms);
		}
	}
	else {
		return 0;
	}
}

void clearSpecifiedDelay(SpecifiedDelay_t * theDelay) {
	theDelay->duration_ms = 0;
	theDelay->startTime_tick = 0;
	theDelay->inProgress = false;
}

