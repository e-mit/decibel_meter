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
}


int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_float2IntFrac2dp);
    return UNITY_END();
}


