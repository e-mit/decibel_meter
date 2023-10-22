// Run tests, in two groups:
//  1) Unit tests for functions which only use standard C
//  2) If STM32_TEST is defined, also run unit/system tests which require STM32 drivers/libraries

#include "test_sound.h"

#ifdef STM32_TEST
#include "test_stm32.h"
#endif

int main(void) {

	run_tests();

	#ifdef STM32_TEST
		run_stm32_tests();
	#endif

	return 0;
}
