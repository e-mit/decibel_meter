// Unit tests for efficient_10log10.c/h using the Unity test framework

#include "../inc/efficient_10log10.h"
#include "../unity/unity.h"

void setUp(void) {}
void tearDown(void) {}

#define ONE_DP_ERROR_LIMIT 0.14f
#define NTEST_INPUTS 23


void test_efficient10log10(void) {
	const uint64_t testInputs[NTEST_INPUTS] = {596, 2178, 5591, 11170, 51992, 242002,
			1126424, 5243060, 24404378, 113592759, 528729520, 2461027526, 11455113150,
			53319036834, 248179101477, 1155175900896, 5376888521506, 25027296838757,
			116492202609360, 542225289299535, 2523845010827701, 11747503785573328,
			54679999999999952};

	const float testTrueOutputs[NTEST_INPUTS] = {27.8f, 33.4f, 37.5f, 40.5f, 47.2f,
			53.8f, 60.5f, 67.2f, 73.9f, 80.6f, 87.2f, 93.9f, 100.6f, 107.3f, 113.9f, 120.6f,
			127.3f, 134.0f, 140.7f, 147.3f, 154.0f, 160.7f, 167.4f};

	int32_t integerPart;
	int32_t fractionalPart;
	for (uint32_t i = 0; i < NTEST_INPUTS; i++) {
		efficient10log10(testInputs[i], &integerPart, &fractionalPart);
		float result = ((float) integerPart) + (((float) fractionalPart)/10.0f);
		TEST_ASSERT_FLOAT_WITHIN(ONE_DP_ERROR_LIMIT, testTrueOutputs[i], result);
	}
}

void test_correctIntFracNumber(void) {
	int32_t integerPart;
	int32_t fractionalPart;

	integerPart = 0;
	fractionalPart = 2;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(0, integerPart);
	TEST_ASSERT_EQUAL_INT32(2, fractionalPart);

    integerPart = -7;
    fractionalPart = -5;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(-7, integerPart);
	TEST_ASSERT_EQUAL_INT32(-5, fractionalPart);

    integerPart = 3;
    fractionalPart = 4;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(3, integerPart);
	TEST_ASSERT_EQUAL_INT32(4, fractionalPart);

    integerPart = 2;
    fractionalPart = 12;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(3, integerPart);
	TEST_ASSERT_EQUAL_INT32(2, fractionalPart);

    integerPart = -2;
    fractionalPart = -21;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(-4, integerPart);
	TEST_ASSERT_EQUAL_INT32(-1, fractionalPart);

    integerPart = 2;
    fractionalPart = -6;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(1, integerPart);
	TEST_ASSERT_EQUAL_INT32(4, fractionalPart);

    integerPart = -10;
    fractionalPart = 22;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(-7, integerPart);
	TEST_ASSERT_EQUAL_INT32(-8, fractionalPart);

    integerPart = 10;
    fractionalPart = -31;
	correctIntFracNumber(&integerPart, &fractionalPart);
	TEST_ASSERT_EQUAL_INT32(6, integerPart);
	TEST_ASSERT_EQUAL_INT32(9, fractionalPart);
}

int main(void) {
	printf("\n#######################\n./%s\n\n",__FILE__);
    UNITY_BEGIN();
    RUN_TEST(test_efficient10log10);
    RUN_TEST(test_correctIntFracNumber);
    int v = UNITY_END();
    printf("-----------------------\n");
    return v;
}


