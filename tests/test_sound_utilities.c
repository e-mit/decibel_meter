// Unit tests for sound_utilities.c/h using the Unity test framework

#include "../inc/sound_utilities.h"
#include "../unity/unity.h"

void setUp(void) {}
void tearDown(void) {}

#define INTFRAC2(i,f) i##.##f
#define INTFRAC(i,f) INTFRAC2(i,f)

void test_float2IntFrac2dp(void) {
	uint32_t intpart;
	uint8_t fracpart2dp;
	#define INTPART 35
	#define FRACPART 67
	float2IntFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 12
	#define FRACPART 0
	float2IntFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 0
	float2IntFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 99
	float2IntFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 5555
	#define FRACPART 01
	float2IntFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 5555
	#define FRACPART 10
	float2IntFrac2dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART
}

void test_float2IntFrac1dp(void) {
	uint32_t intpart;
	uint8_t fracpart2dp;
	#define INTPART 35
	#define FRACPART 6
	float2IntFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 1200
	#define FRACPART 0
	float2IntFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 0
	float2IntFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 0
	#define FRACPART 9
	float2IntFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART

	#define INTPART 5
	#define FRACPART 1
	float2IntFrac1dp(INTFRAC(INTPART, FRACPART), &intpart, &fracpart2dp);
	TEST_ASSERT_EQUAL_UINT32(INTPART, intpart);
	TEST_ASSERT_EQUAL_UINT8(FRACPART, fracpart2dp);
	#undef INTPART
	#undef FRACPART
}


void test_sumToIntAverage(void) {

	uint8_t intpart;
	uint8_t fracpart1dp;

	//sumToIntAverage(&intpart, &fracpart1dp, const int32_t intSum,
	//	             const int32_t frac1dpSum, const uint32_t sumCount);

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


void test_findMinMax(void) {
	//findMinMax(int32_t * min, int32_t * max, const int32_t * array, const uint32_t length);
	int32_t min;
	int32_t max;

	int32_t array[] = {5};
	findMinMax(&min, &max, array, 1);
	TEST_ASSERT_EQUAL_INT32(5, min);
	TEST_ASSERT_EQUAL_INT32(5, max);

}

int main(void) {
	printf("\n#######################\n./%s\n\n",__FILE__);
    UNITY_BEGIN();
    RUN_TEST(test_float2IntFrac2dp);
    RUN_TEST(test_float2IntFrac1dp);
    RUN_TEST(test_sumToIntAverage);
    RUN_TEST(test_findMinMax);
    int v = UNITY_END();
    printf("-----------------------\n");
    return v;
}


