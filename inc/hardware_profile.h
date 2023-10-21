#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "project_config.h"
#include <stdbool.h>

// Interrupt priorities:
// Priority must be number 0-3; , M0+ does not use subpriorities; IRQ number breaks tie
// Equal priority interrupts do not interrupt each other. Lower priorities interrupt higher ones.
// If two equal-priority interrupts are pending, the IRQn breaks the tie.

// NB: Systick is priority 0 with IRQn = -1
#define DMA_IRQ_PRIORITY 2       // IRQn = 9

// timer for debug process timing only:
#define TMR15_IRQ_PRIORITY 2 // IRQn = 20

///////////////////////////////////////////////////////////////////////

#define AHB_CLK_DIV RCC_SYSCLK_DIV1
#define APB1_CLK_DIV RCC_HCLK_DIV1

#define PASSFAIL_STR(x) (x ? "pass" : "fail")

//////////////////////////////////////////////////////////////////////////////////

// UART:

#define USART4_TX_PIN GPIO_PIN_0
#define USART4_TX_PORT GPIOA
#define USART4_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // must agree with USART4_TX_PORT

#define USART4_RX_PIN GPIO_PIN_1
#define USART4_RX_PORT GPIOA
#define USART4_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define UART_BAUD 115200
#define UART_WORDLENGTH UART_WORDLENGTH_8B
#define UART_STOPBITS UART_STOPBITS_1
#define UART_PARITY UART_PARITY_NONE

//////////////////////////////////////////////////////////////////////////////////

extern DMA_HandleTypeDef hdma_spi1_rx;

// I2S1 pins

// Enable an internal pulldown on the SD line so it does not float when the right audio channel is enabled
// this removes the need for an external pulldown.
#define I2S1_SD_INTERNAL_PD

#define I2S1_CLK_PIN GPIO_PIN_5
#define I2S1_CLK_PORT GPIOA
#define I2S1_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // must agree with I2S1_CLK_PORT
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

// functions

void GPIO_Init(void);
void errorHandler(const char * func, uint32_t line, const char * file);
bool SystemClock_Config(void);
const char * getStartupReason(void);
bool UART_Init(void);
void printString(char * str, uint16_t len);

#endif
