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


/** @brief  Check whether the selected RCC flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *            @arg @ref RCC_FLAG_PWRRST BOR or POR/PDR reset
  *            @arg @ref RCC_FLAG_OBLRST OBLRST reset
  *            @arg @ref RCC_FLAG_PINRST Pin reset
  *            @arg @ref RCC_FLAG_SFTRST Software reset
  *            @arg @ref RCC_FLAG_IWDGRST Independent Watchdog reset
  *            @arg @ref RCC_FLAG_WWDGRST Window Watchdog reset
  *            @arg @ref RCC_FLAG_LPWRRST Low Power reset
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */

#ifdef DEBUG_AND_TESTS
extern void printSerial(const char* format, ...);
#endif

void getStartupReason(void) {
#ifdef DEBUG_AND_TESTS
	printSerial("Reset flag(s):\n");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST) == 1) {
		printSerial("	BOR or POR/PDR reset\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) == 1) {
		printSerial("	Option byte loader reset\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == 1) {
		printSerial("	NRST pin reset\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == 1) {
		printSerial("	Software reset\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == 1) {
		printSerial("	Independent Watchdog reset\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) == 1) {
		printSerial("	Window watchdog reset\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == 1) {
		printSerial("	Low power reset\n");
	}
	/* Clear source Reset Flag */
	__HAL_RCC_CLEAR_RESET_FLAGS();
#endif
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

