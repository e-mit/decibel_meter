#include "hardware_profile.h"
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "project_config.h"
#include "UART.h"

static UART_HandleTypeDef uart;

////////////////////////////////////////

bool UART_Init(void) {
	return USART4_UART_Init();
}

bool LPUART1_UART_Init(void) {
	uart.Instance = LPUART1;
	uart.Init.BaudRate = UART_BAUD;
	uart.Init.WordLength = UART_WORDLENGTH;
	uart.Init.StopBits = UART_STOPBITS;
	uart.Init.Parity = UART_PARITY;
	uart.Init.Mode = UART_MODE_TX_RX;
	uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	uart.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&uart) != HAL_OK) {
		return false;
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&uart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
		return false;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&uart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
		return false;
	}
	if (HAL_UARTEx_DisableFifoMode(&uart) != HAL_OK) {
		return false;
	}
	return true;
}

bool USART4_UART_Init(void) {
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

void UARTprint(char * str, uint16_t len) {
	HAL_UART_Transmit(&uart, (uint8_t*)str, len, 0xFFFF);
}

