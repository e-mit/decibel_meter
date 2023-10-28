// Demo program for decibel sound meter project.
// Define TESTS to run system tests, otherwise the program
// will continually print A-weighted sound pressure level (SPL dBA)
// and peak sound amplitude (mPa) over the serial port.

#include <stdbool.h>
#include "hardware_profile.h"
#include "print_functions.h"
#include "sound_measurement.h"


int main(void)
{
	if (HAL_Init() != HAL_OK)
	{
		errorHandler(__func__, __LINE__, __FILE__);
	}
	if (!SystemClock_Config())
	{
		errorHandler(__func__, __LINE__, __FILE__);
	}

	if (!UART_Init())
	{
		errorHandler(__func__, __LINE__, __FILE__);
	}

	if (!soundInit(DMA_Init, I2S1_Init, TIM3_Init, DMA1_Channel1_IRQn))
	{
		errorHandler(__func__, __LINE__, __FILE__);
	}

	#ifdef TESTS
		test_soundSystem();
	#else
		if (!enableMicrophone(true))
		{
			errorHandler(__func__, __LINE__, __FILE__);
		}

		SoundData_t soundData = {0};
		if (!startSPLcalculation())
		{
			errorHandler(__func__, __LINE__, __FILE__);
		}

		while (true)
		{
			if (getSoundData(&soundData, true, true))
			{
				clearMaximumAmplitude();
				print("%u.%u  %u.%02u  %u\n", soundData.SPL_dBA_int,
					  soundData.SPL_dBA_fr_1dp, soundData.peak_amp_mPa_int,
					  soundData.peak_amp_mPa_fr_2dp, soundData.stable);
				if (!startSPLcalculation())
				{
					errorHandler(__func__, __LINE__, __FILE__);
				}
			}
		}
	#endif
	return 0;
}
