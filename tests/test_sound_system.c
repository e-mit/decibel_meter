// System tests designed to run on the target microcontroller.

#include "../inc/print_functions.h"
#include "sound_test_data.h"

#define PC_DIFF_LIMIT 2 // maximum abs() allowable percentage difference
#define SPL_TOL 0.14    // maximum allowable absolute SPL (dB) difference
#define AMP_TOL 0.014   // maximum allowable absolute pressure (mPa) difference

#define TEST_TITLE(name) print("\nTEST: " name "\n")
#define TEST_RESULT(x) print("%s\n-------------------------------------\n", x ? "PASS" : "FAIL");

void test_sound_system(void) {
	print("\n\n-------------------------------------\n");
	print("Starting system tests\n");
	print("-------------------------------------\n\n");

	TEST_TITLE("getFilteredMaxAmplitudeQ31");

	// Use the array x1_32 (length=X1_32_LEN=1024) as input data.
	// First filter/discard the initial X1_32_LEN - NFILT values to initialize the filter, then
	// filter the final NFILT values and find the maximum from this part.

	// the expected result depends on Fs and NFILT (but keep the latter constant)
	#define NFILT 128
	#if (I2S_FREQ == 15625)
		const float max_filt_expected = 192251.0;
	#else
		#error("Fs not implemented yet")
	#endif

	UNUSED(getFilteredMaxAmplitudeQ31(x1_32, X1_32_LEN - NFILT, true, true));
	uint32_t filtmax = getFilteredMaxAmplitudeQ31(&(x1_32[X1_32_LEN - NFILT]), NFILT, false, true);
	float pcDiff = (100.0*(((float) filtmax) - max_filt_expected))/max_filt_expected;
	print("   filtmax = %.2f\n", ((float) filtmax));
	print("   Abs difference = %.2f\n", ((float) filtmax) - max_filt_expected);
	print("   Percentage difference (Q31) = %.2f%%\n", pcDiff);
	TEST_RESULT((pcDiff < PC_DIFF_LIMIT) && (pcDiff > -PC_DIFF_LIMIT));

	/////////////////////////////////////////////////////////////////////////////

	TEST_TITLE("calculateSPLQ31");
	bool ok = true;

	// Use real microphone data to test. Note that x1_32 was acquired with Fs = 31250 Hz
	// But calculated SPLs have been scaled to account for varying I2S_FREQ and FFT_N.
	#if (I2S_FREQ == 31250)
		#if (FFT_N == 128)

		#elif (FFT_N == 256)
			const float SPL_expected = 87.9; //87.909407;
			// {49.18719, 47.61427, 54.74916, 63.08692, 86.66056, 65.99084};
			const float bandSPL_expected[SOUND_FREQ_BANDS] = {49.2, 47.6, 54.7, 63.1, 86.7, 66.0};
		#elif (FFT_N == 512)
			const float SPL_expected = 87.9; //87.947914;
			//{42.72702, 49.50471, 54.34899, 63.27509, 86.69020, 66.20656};
			const float bandSPL_expected[SOUND_FREQ_BANDS] = {42.7, 49.5, 54.3, 63.3, 86.7, 66.2};
		#elif (FFT_N == 1024)
			const float SPL_expected = 88.0; //87.964126;
			// {32.29662, 30.94850, 31.75690, 31.17087, 86.76750, 45.40472};
			const float bandSPL_expected[SOUND_FREQ_BANDS] = {32.3, 30.9, 31.8, 31.2, 86.8, 45.4};
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#elif (I2S_FREQ == 15625)
		#if (FFT_N == 128)
			// values calculated with octave on the first 128 values in x1_32:
			const float SPL_expected = 86.7; //86.693684;
			//{52.83744, 54.53367, 62.10349, 86.72742, 62.75779, 56.55911};
			const float bandSPL_expected[SOUND_FREQ_BANDS] = {52.8, 54.5, 62.1, 86.7, 62.8, 56.6};
		#elif (FFT_N == 256)
			const float SPL_expected = 86.7; //86.687506;
			//{47.61427, 54.74916, 63.08692, 86.66056, 65.99084, 61.29402};
			const float bandSPL_expected[SOUND_FREQ_BANDS] = {47.6, 54.7, 63.1, 86.7, 66.0, 61.3};
		#elif (FFT_N == 512)
			const float SPL_expected = 86.8; //86.750352;
			//{49.50471, 54.34899, 63.27509, 86.69020, 66.20656, 61.22843};
			const float bandSPL_expected[SOUND_FREQ_BANDS] = {49.5, 54.3, 63.3, 86.7, 66.2, 61.2};
		#else
			#error("N-points and/or Fs not implemented yet")
		#endif
	#else
		#error("N-points and/or Fs not implemented yet")
	#endif

	// Values for testing amplitude scaling/output:
	uint32_t maximumAmplitude_in = 875787;
	float peak_amp_mPa_actual = 2945.97; // assuming ik_mPa = 3.3638e-3;

	memcpy((int32_t *) &dataBuffer, &x1_32, 4*FFT_N);
	calculateSPLQ31();
	SoundData_t data;
	spl_sum_count = 1;
	maximumAmplitude = maximumAmplitude_in;
	getSoundData(&data, true, true);

	float theSPL = ((float) data.SPL_dBA_int) + (((float) data.SPL_dBA_fr_1dp)/10.0);
	print("   SPL abs difference = %.2f (SPL = %.2f)\n", fabs(theSPL - SPL_expected), theSPL);
	ok = ok && (fabs(theSPL - SPL_expected) <= SPL_TOL);

	for (uint32_t i = 0; i < SOUND_FREQ_BANDS; i++) {
		theSPL = ((float) bandSPL_int[i]) + (((float) bandSPL_frac_1dp[i])/10.0);
		print("   Band %i abs difference = %.2f\n", i, fabs(theSPL - bandSPL_expected[i]));
		ok = ok && (fabs(theSPL - bandSPL_expected[i]) <= SPL_TOL);
	}

	float theAmp = ((float) data.peak_amp_mPa_int) + (((float) data.peak_amp_mPa_fr_2dp)/100.0);
	print("   Max. amplitude abs difference = %.2f\n", fabs(theAmp - peak_amp_mPa_actual));
	ok = ok && (fabs(theAmp - peak_amp_mPa_actual) <= AMP_TOL);

	TEST_RESULT(ok);

	print("-------------------------------------\n\n");
}
