

#ifdef DEBUG_AND_TESTS
	#define NTIMES (4*2)
	#define N_SPL_SAVE 250

	static volatile float32_t SPL = 0.0;
	static volatile float32_t bandSPL[SOUND_FREQ_BANDS] = {0.0};
	volatile uint32_t NhalfBuffersCmpltd = 0, NhalfBufLimit = 0;
	volatile bool autoStopI2S = false;
	volatile uint32_t nspl = 0;
	volatile int32_t SPL_intBuf[N_SPL_SAVE] = {0}; // this will save the first N_SPL_SAVE SPL values


// to get an array (FFT_N values) of amplitude data after a specific number of half-buffers have elapsed:
// 1) MUST disable SPL_calc because this overwrites the dataBuffer array
// 2) set autoStopI2S = true
// 3) set global variable "NhalfBufLimit" to the desired number
// 4) enable I2S.
// 5) Then use this function to check for completion and to get the array of extracted values.
int32_t * stopI2S_afterNhalfBuffers(void) {
	if (NhalfBuffersCmpltd >= NhalfBufLimit) {
		return (int32_t *) dataBuffer;
	}
	else {
		return NULL;
	}
}

bool soundUnitTests(void) {

	#define PC_DIFF_LIMIT 2 // maximum abs() allowable percentage difference
	volatile uint32_t time_us = 0;
	volatile float pcDiff = 0.0;
	volatile float volFloat = 0.0;

	bool testsOK = true, ok = true;

	print("\nSound unit tests on %s for %s\n",__DATE__,__FILE__);
	print("FFT_N = %u, I2S_FREQ = %u\n\n",FFT_N, I2S_FREQ);

	// decodeI2SdataLch: convert input raw I2S data into signed 32 bit numbers, assuming the I2S data is Left
	// channel only and the first datum starts at element 0.
	/*Left channel is received first (sound data are on left channel
	when mic sel pin is low). Thus the first few elements of the data
	buffer end up as:
	Left1H, Left1L, Right1H, Right1L, Left2H, Left2L, Right2H, Right2L,...
	where L/H = low/high 16-bits of the 32-bit data value
	and 1,2,3 = sample number.*/
	#define nsamp 4
	uint16_t inBuf[nsamp*4] = {0x7FF1, 0xE800, 0, 0, 0xB202, 0xB900, 0, 0, 65157, 16384, 0, 0, 1, 1, 0, 0};
	uint32_t inBuflen = nsamp*4;
	int32_t outBuf[nsamp] = {0};
	const int32_t outBufExpected[nsamp] = {8385000, -5111111, -96960, 256};
	decodeI2SdataLch(inBuf, inBuflen, outBuf);
	ok = true;
	for (uint32_t i=0; i<nsamp; i++) {
		//print("EXP: %i, OBT: %i\n",outBufExpected[i], outBuf[i]);
		ok = ok && (outBufExpected[i] == outBuf[i]);
	}
	print("decodeI2SdataLch: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	// findMinMax

	int32_t min, max;
	#define nn 5
	int32_t array[nn] = {(INT32_MIN+2), 99999999, -222222, 0, 56};
	const int32_t maxExpected = 99999999;
	const int32_t minExpected = (INT32_MIN+2);
	findMinMax(&min, &max, array, nn);
	ok = true;
	ok = ok && (max == maxExpected);
	ok = ok && (min == minExpected);
	print("findMinMax: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	//getPo2factor
	// find the largest positive integer bitshift m, such that: smallVal*(2^m) <= bigVal
	// this is the largest upward bitshift that can be applied to smallVal such
	// that it is still <= bigVal.
	// there is no checking for erroneous inputs (e.g. bigVal < smallVal)

	ok = true;
	ok = ok && (getPo2factor(16, 2) == 3);
	ok = ok && (getPo2factor(16, 3) == 2);
	ok = ok && (getPo2factor(16, 0) == 0);
	ok = ok && (getPo2factor(2,  2) == 0);
	ok = ok && (getPo2factor(24, 3) == 3);
	print("getPo2factor: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	//getFilteredMaxAmplitude
	// apply a simple single-pole hi-pass IIR filter to the incoming data to remove the dc offset,
	// and return the largest +ve amplitude from this input array. Called on the decoded I2S values.

	// as input data, use the array x1_32 (length=X1_32_LEN=1024).
	// First filter the initial X1_32_LEN - NFILT values so that the filter state
	// becomes less dependent on the previous memory (although this test function should be run
	// at start of main anyway, so state is the initial state).
	// Then filter the final NFILT values and return the maximum from this batch (and time this too).

	// the expected result depends on Fs and NFILT (but keep the latter constant)
	#define NFILT 128
	#if (I2S_FREQ == 15625)
		const float max_filt_expected = 176936.800155;
	#elif (I2S_FREQ == 31250)
		const float max_filt_expected = 123698.439946;
	#else
		#error("Fs not implemented yet")
	#endif

	volatile uint32_t fma_ignore = getFilteredMaxAmplitude(x1_32, X1_32_LEN - NFILT); // ignore return value
	fma_ignore++; // to prevent unused warning
	RESET_TMR15_AND_FLAG;
	volatile uint32_t filtmax = getFilteredMaxAmplitude(&(x1_32[X1_32_LEN - NFILT]), NFILT);
	GET_TIME_TMR15(time_us);
	print("Time for filtering (float) is: %u us\n",time_us);
	pcDiff = (100.0*(((float) filtmax) - max_filt_expected))/max_filt_expected;
	print("Percentage difference (float) = %.2f%%\n",pcDiff);
	ok = ((pcDiff < PC_DIFF_LIMIT)&&(pcDiff > -PC_DIFF_LIMIT));
	print("getFilteredMaxAmplitude: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	// now do Q31 version

	fma_ignore = getFilteredMaxAmplitudeQ31(x1_32, X1_32_LEN - NFILT, false, true); // ignore return value
	RESET_TMR15_AND_FLAG;
	filtmax = getFilteredMaxAmplitudeQ31(&(x1_32[X1_32_LEN - NFILT]), NFILT, false, true);
	GET_TIME_TMR15(time_us);
	print("Time for filtering (Q31) is: %u us\n",time_us);
	pcDiff = (100.0*(((float) filtmax) - max_filt_expected))/max_filt_expected;
	print("Percentage difference (Q31) = %.2f%%\n",pcDiff);
	ok = ((pcDiff < PC_DIFF_LIMIT)&&(pcDiff > -PC_DIFF_LIMIT));
	print("getFilteredMaxAmplitudeQ31: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	/////////////////////////////////////////////////////////////////////////////

	// calculateSPL()

	// input: dataBuffer which must contain FFT_N values
	// dataBuffer must be writable as it is used for storage etc throughout this function.
	// the SPL result is returned and global bandSPL[] is also modified

	// use x1_32 as input data BUT the results also depend on FFT_N and Fs
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

	print("calculateSPL() test\n");

	memcpy((int32_t *) &dataBuffer, &x1_32, 4*FFT_N); // 3rd arg is #bytes

	RESET_TMR15_AND_FLAG;
	calculateSPL();
	GET_TIME_TMR15(time_us);
	// convert the result to 1.d.p for a fair test:
	SPL = roundf(SPL*10.0)/10.0;

	print("	Time taken for SPL calc is: %u us\n",time_us);
	print("	SPL abs difference = %.2f\n",SPL - SPL_expected);

	for (uint32_t i = 0; i<SOUND_FREQ_BANDS; i++) {
		// convert the result to 1.d.p for a fair test:
		bandSPL[i] = roundf(bandSPL[i]*10.0)/10.0;
		print("	Band %i abs difference = %.2f\n",i,bandSPL[i] - bandSPL_expected[i]);
	}
	print("\n");

	////////////////////////////

	print("calculateSPLQ31() test\n");

	memcpy((int32_t *) &dataBuffer, &x1_32, 4*FFT_N); // 3rd arg is #bytes

	RESET_TMR15_AND_FLAG;
	calculateSPLQ31();
	GET_TIME_TMR15(time_us);

	float theSPL = ((float) SPL_int) + (((float) SPL_frac_1dp)/10.0);

	print("	Time taken for SPL calc is: %u us\n",time_us);
	print("	SPL abs difference = %.2f\n",theSPL - SPL_expected);

	for (uint32_t i = 0; i<SOUND_FREQ_BANDS; i++) {
		theSPL = ((float) bandSPL_int[i]) + (((float) bandSPL_frac_1dp[i])/10.0);
		//print("	Band %i SPL = %i.%i = %.1fF, expected: %.1f\n",
		//		i,bandSPL_int[i],bandSPL_frac_1dp[i],theSPL,bandSPL_expected[i]);
		print("	Band %i abs difference = %.2f\n",i,theSPL - bandSPL_expected[i]);
	}
	print("\n");

	/////////////////////////////////////////////////////////////////////////////

	// SPL values of 30 - 90 dB require log10() on numbers between 1.72922e9 -> 1.72922e15
	#define nlogs 5
	volatile float input[nlogs] = {1.72922e9, 3.7676e10, 9.0e12, 1.9283e14, 1.72922e15};
	volatile float exp_ans[nlogs] = {9.23785025, 10.57606, 12.9542425, 14.2851746, 15.23785025};
	for (uint32_t i=0; i<nlogs; i++) {
		RESET_TMR15_AND_FLAG;
		volatile float ans = log10(input[i]);
		GET_TIME_TMR15(time_us);
		pcDiff = (100.0*(ans - exp_ans[i]))/exp_ans[i];
		print("log10() percentage difference = %.2f%%\n",pcDiff);
		print("Time for log10(): %u us\n",time_us);
	}

	print("\n");
	volatile float A = 3.141592e2;
	volatile float B = 29.762e10;
	volatile bool comp = false;
	RESET_TMR15_AND_FLAG;
	if (B > A) {
		comp = true;
	}
	GET_TIME_TMR15(time_us);
	if (!comp) {
		print("Float compare FAILED\n");
	}
	else {
		print("Time for a float compare: %u us\n",time_us);
	}

	print("\n");
	volatile float C = 7.141592e2;
	volatile float exp_ACans = 1028.3184;
	RESET_TMR15_AND_FLAG;
	volatile float ACans = A+C;
	GET_TIME_TMR15(time_us);
	pcDiff = (100.0*(ACans - exp_ACans))/exp_ACans;
	print("Float addition percentage difference = %.2f%%\n",pcDiff);
	print("Time for a float addition: %u us\n",time_us);

	print("\n");
	volatile float exp_AmCans = 224359.6829;
	RESET_TMR15_AND_FLAG;
	volatile float AmCans = A*C;
	GET_TIME_TMR15(time_us);
	pcDiff = (100.0*(AmCans - exp_AmCans))/exp_AmCans;
	print("Float multiplication percentage difference = %.2f%%\n",pcDiff);
	print("Time for a float multiplication: %u us\n",time_us);

	print("\n");
	volatile float exp_CdAans = 2.27323981;
	RESET_TMR15_AND_FLAG;
	volatile float CdAans = C/A;
	GET_TIME_TMR15(time_us);
	pcDiff = (100.0*(CdAans - exp_CdAans))/exp_CdAans;
	print("Float division percentage difference = %.2f%%\n",pcDiff);
	print("Time for a float division: %u us\n",time_us);

	print("\n");
	volatile uint64_t u64 = UINT64_C(0x40A0900F04004000);
	RESET_TMR15_AND_FLAG;
	volatile float u64f = (float) u64;
	GET_TIME_TMR15(time_us);
	u64f+=1.0; // just so it is used
	print("Time for a u64->float cast: %u us\n",time_us);

	/////////////////////////////////////////////////////////////////////////////

	// efficient 10log10
	// NB: GCC compiler memory barrier is:   asm volatile("" ::: "memory");

	print("\n");
	print("efficient_10log10() tests\n");

	volatile int32_t integerPart, fractionalPart;

	for (uint32_t i=0; i<Ntest_inputs_10log10; i++) {
		RESET_TMR15_AND_FLAG;
		efficient_10log10(testInputs[i], (int32_t *) &integerPart, (int32_t *) &fractionalPart);
		GET_TIME_TMR15(time_us);
		print("efficient_10log10() ran in = %u us\n",time_us);
		float result = ((float) integerPart) + (((float) fractionalPart)/10.0);
		float absErr = result - testTrueOutputs[i];
		print("                    absErr = %.2f\n",absErr);

		RESET_TMR15_AND_FLAG;
		volFloat = 10.0*log10((float) testInputs[i]);
		GET_TIME_TMR15(time_us);
		print("float version ran in = %u us\n",time_us);
		// convert the result to 1.d.p for a fair test:
		volFloat = roundf(volFloat*10.0)/10.0;
		print("        float absErr = %.2f\n",volFloat - testTrueOutputs[i]);
	}

	print("\n");

	///////////////////////////////////////////////////////

	print("correctIntFracNumber() tests:\n");

	ok = true;
	integerPart = 0;
	fractionalPart = 2;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 0) && (fractionalPart == 2);

	integerPart = -7;
	fractionalPart = -5;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == -7) && (fractionalPart == -5);

	integerPart = 3;
	fractionalPart = 4;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 3) && (fractionalPart == 4);

	integerPart = 2;
	fractionalPart = 12;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 3) && (fractionalPart == 2);

	integerPart = -2;
	fractionalPart = -21;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == -4) && (fractionalPart == -1);

	integerPart = 2;
	fractionalPart = -6;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 1) && (fractionalPart == 4);

	integerPart = -10;
	fractionalPart = 22;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == -7) && (fractionalPart == -8);

	integerPart = 10;
	fractionalPart = -31;
	print("%i,%i gives: ",integerPart,fractionalPart);
	correctIntFracNumber((int32_t *) &integerPart, (int32_t *) &fractionalPart);
	print("%i.%i\n",integerPart,fractionalPart);
	ok = ok && (integerPart == 6) && (fractionalPart == 9);

	print("correctIntFracNumber: %s\n\n", PASSFAIL_STR(ok));
	testsOK = testsOK && ok;

	print("\n");

	///////////////////////////////////////////////////////

	return testsOK;
}

#endif
