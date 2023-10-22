// Hardware definition, setup and support functions.
// This is device-dependent and defines specific pins for I/O etc.
// Also see stm32g0xx_hal_msp.c for initialisation functions used by the
// Hardware Abstraction Layer (HAL) MCU Support Package (MSP).

#include "hardware_profile.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "project_config.h"
#include "print_functions.h"

static UART_HandleTypeDef uart;
TIM_HandleTypeDef settleTimer;
I2S_HandleTypeDef i2s1;

////////////////////////////////////////

// Call this as: errorHandler(__func__, __LINE__, __FILE__);
void errorHandler(const char * func, uint32_t line, const char * file) {
	print("Error in %s at line %u in file: %s\n", func, line, file);
	while (true) {
	}
}

// Set up all system clocks.
bool SystemClock_Config(void) {
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

	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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

// Set up UART for printing general messages over serial.
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


// Initialize TIMER3 but do not start it.
// Return bool success.
bool TIM3_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	settleTimer.Instance = TIM3;
	#if (TMR3_PRESCALER > 65535)
		#error("TMR3 prescaler must be a 16-bit number")
	#endif
	settleTimer.Init.Prescaler = TMR3_PRESCALER;
	settleTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	#if (TMR3_PERIOD > 65535)
		#error("TMR3 period must be a 16-bit number")
	#endif
	settleTimer.Init.Period = TMR3_PERIOD;
	settleTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	settleTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&settleTimer) != HAL_OK) {
		return false;
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&settleTimer, &sClockSourceConfig) != HAL_OK) {
		return false;
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&settleTimer, &sMasterConfig) != HAL_OK) {
		return false;
	}

	// Initialising the time base causes the UIF flag to get set: clear it.
	__HAL_TIM_CLEAR_FLAG(&settleTimer, TIM_SR_UIF);
	return true;
}

// Initialize I2S but do not enable it.
// Return bool success.
bool I2S1_Init(void) {
	i2s1.Instance = SPI1;
	i2s1.Init.Mode = I2S_MODE_MASTER_RX;
	i2s1.Init.Standard = I2S_STANDARD_PHILIPS;
	i2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
	i2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	i2s1.Init.AudioFreq = I2S_AUDIOFREQ;
	i2s1.Init.CPOL = I2S_CPOL_LOW;
	return (HAL_I2S_Init(&i2s1) == HAL_OK);
}


// Provide a print interface for print_functions.
void printString(char * str, uint16_t len) {
	HAL_UART_Transmit(&uart, (uint8_t*)str, len, 0xFFFF);
}


