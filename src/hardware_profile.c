#include "hardware_profile.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "project_config.h"
#include "print_functions.h"

// global objects

TIM_HandleTypeDef htim15; // gen purp debug timer
TIM_HandleTypeDef htim16; // gen purp debug timer
TIM_HandleTypeDef htim17; // gen purp debug timer

volatile bool TIM15_flag = false;
volatile bool TIM16_flag = false;
volatile bool TIM17_flag = false;
volatile uint32_t TIM17_rollover_count = 0;

////////////////////////////////////////

void GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	#ifndef HAL_UART_MODULE_ENABLED
		// UART not used, so set its TX pin to the default unused state
		USART4_TX_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin = USART4_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(USART4_TX_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(USART4_TX_PORT, USART4_TX_PIN, GPIO_PIN_RESET);
	#endif

	#ifdef NUCLEO_BOARD
		// LED
		ERROR_LED_GPIO_CLK_ENABLE();
		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = ERROR_LED_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(ERROR_LED_GPIO_Port, &GPIO_InitStruct);

		// Blue button
		BLUE_BUTTON_GPIO_CLK_ENABLE();
		#ifndef ENABLE_BLUE_BUTTON_INT
			GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = BLUE_BUTTON_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);
		#else
			#if ((BLUE_BUTTON_IRQ_PRIORITY > 3)||(BLUE_BUTTON_IRQ_PRIORITY<0))
				#error("Interrupt priority must be 0-3")
			#endif
			GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			GPIO_InitStruct.Pull = BLUE_BUTTON_PULL;
			HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);
			HAL_NVIC_SetPriority(BLUE_BUTTON_IRQn, BLUE_BUTTON_IRQ_PRIORITY, 0);
			HAL_NVIC_EnableIRQ(BLUE_BUTTON_IRQn);
		#endif
	#endif
}


// call like Error_Handler(__func__, __LINE__, __FILE__);
void errorHandler(const char * func, uint32_t line, const char * file) {
	#ifdef NUCLEO_BOARD
		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
	#endif
	#ifdef DEBUG_PRINT
		printSerial("Error in %s at line %u in file: %s\n", func, line, file);
	#endif
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

	// Initialize the peripherals clocks
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

// use the predefined value in TMR16_PERIOD_COUNT
bool TIM16_Init(void) {
	#if (TMR16_PERIOD_COUNT > 65535)
		#error("TMR16 period must be a 16-bit number")
	#endif
	if (!TIM16_Init_With_Period_Count(TMR16_PERIOD_COUNT)) {
		return false;
	}
	return true;
}

// use a runtime value:
bool TIM16_Init_With_Period_Count(uint32_t period_count) {

	if (period_count > UINT16_MAX) {
		return false; // TMR16 period must be a 16-bit number
	}
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = TMR16_PRESCALER;
	#if (TMR16_PRESCALER > 65535)
		#error("TMR16 prescaler must be a 16-bit number")
	#endif
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = period_count;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		return false;
	}
	// set priority but do not enable yet - may not be used
	#if ((TMR16_IRQ_PRIORITY > 3)||(TMR16_IRQ_PRIORITY<0))
		#error("Interrupt priority must be 0-3")
	#endif
	HAL_NVIC_SetPriority(TIM16_IRQn, TMR16_IRQ_PRIORITY, 0);

	// Note that initialising the time base causes the UIF flag to get set. Clear it.
	// NB: TIM_FLAG_UPDATE == TIM_SR_UIF. "Update" means rollover.
	__HAL_TIM_CLEAR_FLAG(&htim16, TIM_SR_UIF);

	return true;
}

// use the predefined value in TMR17_PERIOD_COUNT
bool TIM17_Init(void) {
	#if (TMR17_PERIOD_COUNT > 65535)
		#error("TMR17 period must be a 16-bit number")
	#endif
	if (!TIM17_Init_With_Period_Count(TMR17_PERIOD_COUNT)) {
		return false;
	}
	return true;
}

// use a runtime value:
bool TIM17_Init_With_Period_Count(uint32_t period_count) {

	if (period_count > UINT16_MAX) {
		return false; // TMR17 period must be a 16-bit number
	}

	htim17.Instance = TIM17;
	htim17.Init.Prescaler = TMR17_PRESCALER;
	#if (TMR17_PRESCALER > 65535)
		#error("TMR17 prescaler must be a 16-bit number")
	#endif
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = period_count;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		return false;
	}
	// set priority but do not enable yet - may not be used
	#if ((TMR17_IRQ_PRIORITY > 3)||(TMR17_IRQ_PRIORITY<0))
		#error("Interrupt priority must be 0-3")
	#endif
	HAL_NVIC_SetPriority(TIM17_IRQn, TMR17_IRQ_PRIORITY, 0);

	// Note that initialising the time base causes the UIF flag to get set. Clear it.
	// NB: TIM_FLAG_UPDATE == TIM_SR_UIF. "Update" means rollover.
	__HAL_TIM_CLEAR_FLAG(&htim17, TIM_SR_UIF);

	return true;
}

// Blocking read of a button with pullup.
// Will only return once the button has been pressed and released.
// Reads button at 1 ms intervals.
void waitForDebouncedPressAndReleasePU(uint32_t debounce_count, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	uint32_t pressCount = 0;
	while (pressCount < debounce_count) {
		if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
			// pressed (has hardware pullup)
			pressCount++;
		}
		else {
			pressCount = 0;
		}
		HAL_Delay(1);
	}
	pressCount = 0;
	while (pressCount < debounce_count) {
		if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
			// not pressed (has hardware pullup)
			pressCount++;
		}
		else {
			pressCount = 0;
		}
		HAL_Delay(1);
	}
}

// this finds the total number of ticks elapsed on TMR17, even over multiple rollovers
// BUT NOTE: the timer is stopped and so this does not give a perfectly accurate count
uint32_t getTMR17rolloverCountAndReset(void) {
	// stop TMR17
	__HAL_TIM_DISABLE(&htim17);
	// calc the total ticks
	uint32_t total_ticks = (TIM17_rollover_count*(htim17.Init.Period)) + __HAL_TIM_GetCounter(&htim17);
	// reset:
	__HAL_TIM_SetCounter(&htim17,0);
	TIM17_rollover_count = 0;
	__HAL_TIM_ENABLE(&htim17);
	return total_ticks;
}

// get count without stopping
uint32_t getTMR17rolloverCount(void) {

	volatile uint32_t rollovers1 = TIM17_rollover_count;
	volatile uint32_t ticks = __HAL_TIM_GetCounter(&htim17);
	volatile uint32_t rollovers2 = TIM17_rollover_count;

	uint32_t rollovers = 0;

	if (rollovers1 != rollovers2) {
		// need to try and determine whether "ticks" was obtained just
		// before or after the rollover - should be easy
		if (ticks > (UINT16_MAX/2)) {
			// probably came before the rollover
			rollovers = rollovers1;
		}
		else {
			// probably came after the rollover
			rollovers = rollovers2;
		}
	}

	uint32_t total_ticks = (rollovers*(htim17.Init.Period)) + ticks;
	return total_ticks;
}


