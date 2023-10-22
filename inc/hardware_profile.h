// Hardware definition, setup and support functions.
// This is device-dependent and defines specific pins for I/O etc.
// Also see stm32g0xx_hal_msp.c for initialisation functions used by the
// Hardware Abstraction Layer (HAL) MCU Support Package (MSP).

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#include <stdint.h>
#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "sound_measurement.h"

// UART

#define USART4_TX_PIN GPIO_PIN_0
#define USART4_TX_PORT GPIOA
#define USART4_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define USART4_RX_PIN GPIO_PIN_1
#define USART4_RX_PORT GPIOA
#define USART4_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define UART_BAUD 115200
#define UART_WORDLENGTH UART_WORDLENGTH_8B
#define UART_STOPBITS UART_STOPBITS_1
#define UART_PARITY UART_PARITY_NONE

//////////////////////////////////////////////////////////////////////////////////

// I2S1

// Enable an internal pulldown on the SD line so it does not float when the right audio channel is enabled
// this removes the need for an external pulldown.
#define I2S1_SD_INTERNAL_PD

#define I2S1_CLK_PIN GPIO_PIN_5
#define I2S1_CLK_PORT GPIOA
#define I2S1_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S1_CLK_AF_MAP GPIO_AF0_SPI1

#define I2S1_WS_PIN GPIO_PIN_4
#define I2S1_WS_PORT GPIOA
#define I2S1_WS_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S1_WS_AF_MAP GPIO_AF0_SPI1

#define I2S1_SD_PIN GPIO_PIN_2
#define I2S1_SD_PORT GPIOA
#define I2S1_SD_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S1_SD_AF_MAP GPIO_AF0_SPI1

///////////////////////////////////////////////////////////////////////

// TIMER3

#define TMR3_RES_FREQ_KHZ 1  // sets resolution
#define TMR3_PERIOD ((TMR3_RES_FREQ_KHZ*MIC_SETTLING_PERIOD_MS)-1)
#define TMR3_PRESCALER ((SYSCLK_FREQ_HZ/(1000*TMR3_RES_FREQ_KHZ))-1)

///////////////////////////////////////////////////////////////////////

extern DMA_HandleTypeDef dma1;
extern TIM_HandleTypeDef settleTimer;
extern I2S_HandleTypeDef i2s1;

void GPIO_Init(void);
void errorHandler(const char * func, uint32_t line, const char * file);
bool SystemClock_Config(void);
bool UART_Init(void);
void printString(char * str, uint16_t len);
bool TIM3_Init(TIM_HandleTypeDef ** pHandle);
bool I2S1_Init(I2S_HandleTypeDef ** pHandle);
void DMA_Init(DMA_HandleTypeDef ** pHandle);

#endif
