#include "hardware_profile.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "project_config.h"
#include "print_functions.h"

static UART_HandleTypeDef uart;

////////////////////////////////////////

// Call as: errorHandler(__func__, __LINE__, __FILE__);
void errorHandler(const char * func, uint32_t line, const char * file) {
	print("Error in %s at line %u in file: %s\n", func, line, file);
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

const char * getStartupReason(void) {
	const char * reason = "Unknown";
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST) == 1) {
		reason = "BOR or POR/PDR reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) == 1) {
		reason = "Option byte loader reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == 1) {
		reason = "NRST pin reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == 1) {
		reason = "Software reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == 1) {
		reason = "Independent watchdog reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) == 1) {
		reason = "Window watchdog reset";
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == 1) {
		reason = "Low power reset";
	}
	/* Clear source Reset Flag */
	__HAL_RCC_CLEAR_RESET_FLAGS();
	return reason;
}

bool UART_Init(void) {
	uart.Instance = USART4;
	uart.Init.BaudRate = UART_BAUD;
	uart.Init.WordLength = UART_WORDLENGTH;
	uart.Init.StopBits = UART_STOPBITS;
	uart.Init.Parity = UART_PARITY;
	uart.Init.Mode = UART_MODE_TX_RX;
	uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart.Init.OverSampling = UART_OVERSAMPLING_16;
	uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&uart) != HAL_OK) {
		return false;
	}
	return true;
}

void printString(char * str, uint16_t len) {
	HAL_UART_Transmit(&uart, (uint8_t*)str, len, 0xFFFF);
}


