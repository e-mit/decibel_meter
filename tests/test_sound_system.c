// System tests intended to run on the target microcontroller.

#include "../inc/print_functions.h"
#include "sound_test_data.h"

#define PC_DIFF_LIMIT 2.0f // maximum abs() allowable percentage difference
#define SPL_TOL 0.14f      // maximum allowable absolute SPL (dB) difference
#define AMP_TOL 0.014f     // maximum allowable absolute pressure (mPa) difference

#define TEST_TITLE(name) print("\nTEST: " name "\n")
#define TEST_RESULT(x) print("%s\n-------------------------\n", x ? "PASS" : "FAIL")

void test_soundSystem(void)
{
	print("\n\n-------------------------\n");
	print("Starting system tests\n");
	print("-------------------------\n\n");

	TEST_TITLE("getFilteredMaxAmplitudeQ31");

	// Use the array x1_32 (length=X1_32_LEN=1024) as input data.
	// First filter/discard the initial X1_32_LEN - NFILT values to initialize
	// the filter, then filter the final NFILT values and find the maximum
	// from this part.

	// the expected result depends on Fs and NFILT (but keep the latter constant)
	#define NFILT 128
	#if (I2S_FREQ == 15625)
		const float maxFiltExpected = 192251.0f;
	#else
		#error("Fs not implemented yet")
	#endif

	UNUSED(getFilteredMaxAmplitudeQ31(x1_32, X1_32_LEN - NFILT, true, true));
	uint32_t filtMax = getFilteredMaxAmplitudeQ31(&(x1_32[X1_32_LEN - NFILT]),
			                                      NFILT, false, true);
	float pcDiff = (100.0f*(((float) filtMax) - maxFiltExpected))/maxFiltExpected;
	print("   filtMax = %.2f\n", ((float) filtMax));
	print("   Abs difference = %.2f\n", ((float) filtMax) - maxFiltExpected);
	print("   Percentage difference (Q31) = %.2f%%\n", pcDiff);
	TEST_RESULT((pcDiff < PC_DIFF_LIMIT) && (pcDiff > -PC_DIFF_LIMIT));

	/////////////////////////////////////////////////////////////////////////////

	TEST_TITLE("calculateSPLQ31");
	bool ok = true;

	// Use real microphone data to test. Note that x1_32 was acquired with
	// Fs = 31250 Hz but calculated SPLs have been scaled to account for
	// varying I2S_FREQ and FFT_N.
	#if (I2S_FREQ == 31250)
		#if (FFT_N == 256)
			const float SPL_expected = 87.9f;
			const float bandSPL_expected[SOUND_FREQ_BANDS] =
					    {49.2f, 47.6f, 54.7f, 63.1f, 86.7f, 66.0f};
		#elif (FFT_N == 512)
			const float SPL_expected = 87.9f;
			const float bandSPL_expected[SOUND_FREQ_BANDS] =
						{42.7f, 49.5f, 54.3f, 63.3f, 86.7f, 66.2f};
		#elif (FFT_N == 1024)
			const float SPL_expected = 88.0f;
			const float bandSPL_expected[SOUND_FREQ_BANDS] =
						{32.3f, 30.9f, 31.8f, 31.2f, 86.8f, 45.4f};
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#elif (I2S_FREQ == 15625)
		#if (FFT_N == 128)
			const float SPL_expected = 86.7f;
			const float bandSPL_expected[SOUND_FREQ_BANDS] =
						{52.8f, 54.5f, 62.1f, 86.7f, 62.8f, 56.6f};
		#elif (FFT_N == 256)
			const float SPL_expected = 86.7f;
			const float bandSPL_expected[SOUND_FREQ_BANDS] =
						{47.6f, 54.7f, 63.1f, 86.7f, 66.0f, 61.3f};
		#elif (FFT_N == 512)
			const float SPL_expected = 86.8f;
			const float bandSPL_expected[SOUND_FREQ_BANDS] =
						{49.5f, 54.3f, 63.3f, 86.7f, 66.2f, 61.2f};
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif

	// Values for testing amplitude scaling/output:
	uint32_t maximumAmplitude_in = 875787;
	float peak_amp_mPa_actual = 2945.97f; // assuming ik_mPa = 3.3638e-3;

	memcpy((int32_t *) &dataBuffer, &x1_32, 4*FFT_N);
	calculateSPLQ31();
	SoundData_t data;
	spl_sum_count = 1;
	maximumAmplitude = maximumAmplitude_in;
	getSoundData(&data, true, true);

	float theSPL = ((float) data.SPL_dBA_int) + (((float) data.SPL_dBA_fr_1dp)/10.0f);
	print("   SPL abs difference = %.2f (SPL = %.2f)\n",
		  fabs(theSPL - SPL_expected), theSPL);
	ok = ok && (fabs(theSPL - SPL_expected) <= SPL_TOL);

	for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++)
	{
		theSPL = ((float) bandSPL_int[i]) + (((float) bandSPL_frac_1dp[i])/10.0f);
		print("   Band %i abs difference = %.2f\n",
			  i, fabs(theSPL - bandSPL_expected[i]));
		ok = ok && (fabs(theSPL - bandSPL_expected[i]) <= SPL_TOL);
	}

	float theAmp = ((float) data.peak_amp_mPa_int)
			        + (((float) data.peak_amp_mPa_fr_2dp)/100.0f);
	print("   Max. amplitude abs difference = %.2f\n",
		  fabs(theAmp - peak_amp_mPa_actual));
	ok = ok && (fabs(theAmp - peak_amp_mPa_actual) <= AMP_TOL);

	TEST_RESULT(ok);

	print("-------------------------\n\n");
}
