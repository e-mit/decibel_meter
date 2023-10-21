#include "hardware_profile.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "project_config.h"
#include "print_functions.h"

// global objects
TIM_HandleTypeDef htim15;
volatile bool TIM15_flag = false;


// Call as: errorHandler(__func__, __LINE__, __FILE__);
void errorHandler(const char * func, uint32_t line, const char * file) {
	printSerial("Error in %s at line %u in file: %s\n", func, line, file);
	while (true) {
	}
}

bool SystemClock_Config(void) {

	// Configure the main internal regulator output voltage
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	
	#if (SYSCLK_FREQ_HZ == 16000000)
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	#else
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV32; // EM: changed P,Q (unused) from 2,2 to 32,8
		RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
		#if (SYSCLK_FREQ_HZ == 32000000)
			RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
			RCC_OscInitStruct.PLL.PLLN = 8;
			RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
		#elif (SYSCLK_FREQ_HZ == 36000000)
			RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
			RCC_OscInitStruct.PLL.PLLN = 9;
			RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
		#elif (SYSCLK_FREQ_HZ == 40000000)
			RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
			RCC_OscInitStruct.PLL.PLLN = 10;
			RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
		#elif (SYSCLK_FREQ_HZ == 44000000)
			RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
			RCC_OscInitStruct.PLL.PLLN = 11;
			RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
		#elif (SYSCLK_FREQ_HZ == 48000000)
			RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
			RCC_OscInitStruct.PLL.PLLN = 9;
			RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
		#elif (SYSCLK_FREQ_HZ == 56000000)
			RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
			RCC_OscInitStruct.PLL.PLLN = 14;
			RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
		#elif (SYSCLK_FREQ_HZ == 64000000)
			RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
			RCC_OscInitStruct.PLL.PLLN = 8;
			RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
		#else
			#error("Unrecognised sysclk frequency")
		#endif
	#endif
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		return false;
	}

	// Initialize the CPU, AHB and APB bus clocks
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;

	RCC_ClkInitStruct.AHBCLKDivider = AHB_CLK_DIV;
	RCC_ClkInitStruct.APB1CLKDivider = APB1_CLK_DIV;
	#if (SYSCLK_FREQ_HZ == 16000000)
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
	#elif ((SYSCLK_FREQ_HZ == 32000000) || (SYSCLK_FREQ_HZ == 36000000) || \
		   (SYSCLK_FREQ_HZ == 40000000) || (SYSCLK_FREQ_HZ == 44000000) || \
		   (SYSCLK_FREQ_HZ == 48000000))
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
	#elif ((SYSCLK_FREQ_HZ == 56000000) || (SYSCLK_FREQ_HZ == 64000000))
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
	#else
		#error("Unrecognised sysclk rate")
	#endif
		return false;
	}

	// Initialize the peripheral clocks
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S1|RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2s1ClockSelection = RCC_I2S1CLKSOURCE_SYSCLK; // use HSI directly; not PLL
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		return false;
	}
	return true;
}


bool TIM15_Init(void) {
	#if (TMR15_PERIOD_COUNT > 65535)
		#error("TMR15 period must be a 16-bit number")
	#endif
	if (!TIM15_Init_With_Period_Count(TMR15_PERIOD_COUNT)) {
		return false;
	}
	return true;
}

bool TIM15_Init_With_Period_Count(uint32_t period_count) {

	if (period_count > UINT16_MAX) {
		return false; // TMR15 period must be a 16-bit number
	}

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim15.Instance = TIM15;
	htim15.Init.Prescaler = TMR15_PRESCALER;
	#if (TMR15_PRESCALER > 65535)
		#error("TMR15 prescaler must be a 16-bit number")
	#endif
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = period_count;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
		return false;
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
		return false;
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) {
		return false;
	}
	// set priority but do not enable yet - may not be used
	#if ((TMR15_IRQ_PRIORITY > 3)||(TMR15_IRQ_PRIORITY<0))
		#error("Interrupt priority must be 0-3")
	#endif
	HAL_NVIC_SetPriority(TIM15_IRQn, TMR15_IRQ_PRIORITY, 0);

	// Note that initialising the time base causes the UIF flag to get set. Clear it.
	// NB: TIM_FLAG_UPDATE == TIM_SR_UIF. "Update" means rollover.
	__HAL_TIM_CLEAR_FLAG(&htim15, TIM_SR_UIF);

	return true;
}

