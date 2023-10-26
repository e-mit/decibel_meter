// Unit tests for sound_utilities.c/h using the Unity test framework

#include "../inc/sound_utilities.h"
#include "../unity/unity.h"

void setUp(void) {}
void tearDown(void) {}

#define INTFRAC2(i, f) i##.##f
#define INTFRAC(i, f) INTFRAC2(i, f)

void test_floatToIntAndFrac2dp(void)
{
	uint32_t intpart;
	uint8_t fracpart2dp;
	#define INTPART 35
	#define FRACPART 67
	floatToIntAndFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 12
	#define FRACPART 0
	floatToIntAndFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 0
	floatToIntAndFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 99
	floatToIntAndFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 5555
	#define FRACPART 01
	floatToIntAndFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 5555
	#define FRACPART 10
	floatToIntAndFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART
}

void test_floatToIntAndFrac1dp(void)
{
	uint32_t intpart;
	uint8_t fracpart2dp;
	#define INTPART 35
	#define FRACPART 6
	floatToIntAndFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 1200
	#define FRACPART 0
	floatToIntAndFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 0
	floatToIntAndFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 9
	floatToIntAndFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 5
	#define FRACPART 1
	floatToIntAndFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART
}


void test_sumToIntAverage(void)
{
	uint8_t intpart;
	uint8_t fracpart1dp;

	sumToIntAverage(&intpart, &fracpart1dp, 0, 0, 1);
	TEST_ASSERT_EQUAL_UINT8(0, intpart);
	TEST_ASSERT_EQUAL_UINT8(0, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 0, 1, 1);
	TEST_ASSERT_EQUAL_UINT8(0, intpart);
	TEST_ASSERT_EQUAL_UINT8(1, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 1, 0, 1);
	TEST_ASSERT_EQUAL_UINT8(1, intpart);
	TEST_ASSERT_EQUAL_UINT8(0, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 5, 5, 1);
	TEST_ASSERT_EQUAL_UINT8(5, intpart);
	TEST_ASSERT_EQUAL_UINT8(5, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 5555, 5, 1);
	TEST_ASSERT_EQUAL_UINT8(UINT8_MAX, intpart);
	TEST_ASSERT_EQUAL_UINT8(9, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 500, 20, 10);
	TEST_ASSERT_EQUAL_UINT8(50, intpart);
	TEST_ASSERT_EQUAL_UINT8(2, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 500, 200, 10);
	TEST_ASSERT_EQUAL_UINT8(52, intpart);
	TEST_ASSERT_EQUAL_UINT8(0, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 700, 280, 7);
	TEST_ASSERT_EQUAL_UINT8(104, intpart);
	TEST_ASSERT_EQUAL_UINT8(0, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 2340, 126, 34);
	TEST_ASSERT_EQUAL_UINT8(69, intpart);
	TEST_ASSERT_EQUAL_UINT8(2, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 35, 126, 123);
	TEST_ASSERT_EQUAL_UINT8(0, intpart);
	TEST_ASSERT_EQUAL_UINT8(4, fracpart1dp);

	sumToIntAverage(&intpart, &fracpart1dp, 3, 1, 123);
	TEST_ASSERT_EQUAL_UINT8(0, intpart);
	TEST_ASSERT_EQUAL_UINT8(0, fracpart1dp);
}


void test_findMinMax(void)
{
	{
	int32_t min, max, array[] = {5};
	findMinMax(&min, &max, array, 1);
	TEST_ASSERT_EQUAL_INT32(5, min);
	TEST_ASSERT_EQUAL_INT32(5, max);
	}
	{
	int32_t min, max, array[] = {2, 5};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(2, min);
	TEST_ASSERT_EQUAL_INT32(5, max);
	}
	{
	int32_t min, max, array[] = {5, -2};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(-2, min);
	TEST_ASSERT_EQUAL_INT32(5, max);
	}
	{
	int32_t min, max, array[] = {-5, -2};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(-5, min);
	TEST_ASSERT_EQUAL_INT32(-2, max);
	}
	{
	int32_t min, max, array[] = {0, -2};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(-2, min);
	TEST_ASSERT_EQUAL_INT32(0, max);
	}
	{
	int32_t min, max, array[] = {5, 0};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(0, min);
	TEST_ASSERT_EQUAL_INT32(5, max);
	}
	{
	int32_t min, max, array[] = {6, 6, 6};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(6, min);
	TEST_ASSERT_EQUAL_INT32(6, max);
	}
	{
	int32_t min, max, array[] = {-1, 10, 20};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(-1, min);
	TEST_ASSERT_EQUAL_INT32(20, max);
	}
	{
	int32_t min, max, array[] = {9999, 444, 555};
	findMinMax(&min, &max, array, (sizeof array)/(sizeof array[0]));
	TEST_ASSERT_EQUAL_INT32(444, min);
	TEST_ASSERT_EQUAL_INT32(9999, max);
	}
}

void test_getPo2factor(void)
{
	TEST_ASSERT_EQUAL_UINT32(0, getPo2factor(2, 10));
	TEST_ASSERT_EQUAL_UINT32(0, getPo2factor(10, 10));
	TEST_ASSERT_EQUAL_UINT32(0, getPo2factor(11, 10));
	TEST_ASSERT_EQUAL_UINT32(0, getPo2factor(11, 0));

	TEST_ASSERT_EQUAL_UINT32(2, getPo2factor(8, 2));
	TEST_ASSERT_EQUAL_UINT32(1, getPo2factor(8, 3));
	TEST_ASSERT_EQUAL_UINT32(3, getPo2factor(8, 1));

	TEST_ASSERT_EQUAL_UINT32(11, getPo2factor(34290, 9));
	TEST_ASSERT_EQUAL_UINT32(14, getPo2factor(96732581, 3129));
}

void test_amplitudeDN_to_mPa(void)
{
	float ik_mPa = 1.23e-3f;
	uint16_t intAmp_mPa;
	uint8_t frac2dpAmp_mPa;

	amplitudeDN_to_mPa(0, ik_mPa, &intAmp_mPa, &frac2dpAmp_mPa);
	TEST_ASSERT_EQUAL_UINT16(0, intAmp_mPa);
	TEST_ASSERT_EQUAL_UINT8(0, frac2dpAmp_mPa);

	amplitudeDN_to_mPa(3, ik_mPa, &intAmp_mPa, &frac2dpAmp_mPa);
	TEST_ASSERT_EQUAL_UINT16(0, intAmp_mPa);
	TEST_ASSERT_EQUAL_UINT8(0, frac2dpAmp_mPa);

	amplitudeDN_to_mPa(10, ik_mPa, &intAmp_mPa, &frac2dpAmp_mPa);
	TEST_ASSERT_EQUAL_UINT16(0, intAmp_mPa);
	TEST_ASSERT_EQUAL_UINT8(1, frac2dpAmp_mPa);

	amplitudeDN_to_mPa(100, ik_mPa, &intAmp_mPa, &frac2dpAmp_mPa);
	TEST_ASSERT_EQUAL_UINT16(0, intAmp_mPa);
	TEST_ASSERT_EQUAL_UINT8(12, frac2dpAmp_mPa);

	amplitudeDN_to_mPa(767879, ik_mPa, &intAmp_mPa, &frac2dpAmp_mPa);
	TEST_ASSERT_EQUAL_UINT16(944, intAmp_mPa);
	TEST_ASSERT_EQUAL_UINT8(49, frac2dpAmp_mPa);
}


void test_scaleSPL(void)
{
	int32_t SPLintegerPart;
	int32_t SPLfractionalPart;

	scaleSPL(99999, 0, 0, 0, 0, &SPLintegerPart, &SPLfractionalPart);
	TEST_ASSERT_EQUAL_INT32(50, SPLintegerPart);
	TEST_ASSERT_EQUAL_INT32(0, SPLfractionalPart);

	scaleSPL(99999, 21, 2, 0, 0, &SPLintegerPart, &SPLfractionalPart);
	TEST_ASSERT_EQUAL_INT32(71, SPLintegerPart);
	TEST_ASSERT_EQUAL_INT32(2, SPLfractionalPart);

	scaleSPL(99999, -55, -3, 0, 0, &SPLintegerPart, &SPLfractionalPart);
	TEST_ASSERT_EQUAL_INT32(-5, SPLintegerPart);
	TEST_ASSERT_EQUAL_INT32(-3, SPLfractionalPart);

	scaleSPL(99999, -55, -3, 20, 3, &SPLintegerPart, &SPLfractionalPart);
	TEST_ASSERT_EQUAL_INT32(15, SPLintegerPart);
	TEST_ASSERT_EQUAL_INT32(0, SPLfractionalPart);

	scaleSPL(12522875, 28, 2, -12, -7, &SPLintegerPart, &SPLfractionalPart);
	TEST_ASSERT_EQUAL_INT32(86, SPLintegerPart);
	TEST_ASSERT_EQUAL_INT32(5, SPLfractionalPart);
}

void test_decodeI2SdataLch(void)
{
	#define NSAMP 5
	uint16_t inBuf[NSAMP*4] = {0x7FF1, 0xE800, 0, 0, 0xB202, 0xB900, 0, 0,
			                   65157, 16384, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0};
	int32_t outBuf[NSAMP];
	const int32_t outBufExpected[NSAMP] = {8385000, -5111111, -96960, 256, 0};
	decodeI2SdataLch(inBuf, NSAMP*4, outBuf);
	TEST_ASSERT_EQUAL_INT32_ARRAY(outBufExpected, outBuf, NSAMP);
	#undef NSAMP
}

int main(void)
{
	printf("\n#######################\n./%s\n\n",__FILE__);
    UNITY_BEGIN();
    RUN_TEST(test_floatToIntAndFrac2dp);
    RUN_TEST(test_floatToIntAndFrac1dp);
    RUN_TEST(test_sumToIntAverage);
    RUN_TEST(test_findMinMax);
    RUN_TEST(test_getPo2factor);
    RUN_TEST(test_amplitudeDN_to_mPa);
    RUN_TEST(test_scaleSPL);
    RUN_TEST(test_decodeI2SdataLch);
    int v = UNITY_END();
    printf("-----------------------\n");
    return v;
}


