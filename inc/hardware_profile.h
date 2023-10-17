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

// timers for debug process timing only:
#define TMR15_IRQ_PRIORITY 2 // IRQn = 20
#define TMR16_IRQ_PRIORITY 2 // IRQn = 21
#define TMR17_IRQ_PRIORITY 2 // IRQn = 22

#define TMR14_IRQ_PRIORITY 3   // used for PPD42. IRQn = 19
#define TMR3_IRQ_PRIORITY 3    // used for microphone warmup time. IRQn = 16

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

#ifdef NUCLEO_BOARD
	// The built-in green LED
	#define ERROR_LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
	#define ERROR_LED_Pin GPIO_PIN_5
	#define ERROR_LED_GPIO_Port GPIOA

	// The built-in blue button:
	#define BLUE_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
	#define BLUE_BUTTON_PULL GPIO_NOPULL
	#define BLUE_BUTTON_Pin GPIO_PIN_13
	#define BLUE_BUTTON_GPIO_Port GPIOC
	#define BLUE_BUTTON_IRQn EXTI4_15_IRQn
	#define BLUE_BUTTON_IRQ_PRIORITY 3 // has IRQn = 7
#endif

///////////////////////////////////////////////////////////////////////

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

// general purpose timers:

// TMR15: 1us tick, up to 65.5 ms
#define TMR15_FREQ_HZ 1000000  // sets resolution of the time measurement
#define TMR15_PRESCALER ((SYSCLK_FREQ_HZ/TMR15_FREQ_HZ)-1)
#define TMR15_PERIOD_COUNT 65535

// TMR16: 1ms tick; up to 65.5 sec
#define TMR16_FREQ_HZ 1000
#define TMR16_PRESCALER ((SYSCLK_FREQ_HZ/TMR16_FREQ_HZ)-1)
#define TMR16_PERIOD_COUNT 65535

// TMR17: 0.1ms tick; up to 6.55 sec
#define TMR17_FREQ_HZ 10000
#define TMR17_PRESCALER ((SYSCLK_FREQ_HZ/TMR17_FREQ_HZ)-1)
#define TMR17_PERIOD_COUNT 65535

extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

// NB: use these timers like:
// 1) call TIM15_Init()
// 2) start the timer with: HAL_TIM_Base_Start(&htim15) or HAL_TIM_Base_Start_IT(&htim15)
// 3) to time a section, do:
//    __HAL_TIM_SetCounter(&htim15,0); or RESET_TMR15_AND_FLAG
// 4) ...bit to be timed: may need a volatile variable here...
// 5) get the time with: uint32_t count = __HAL_TIM_GetCounter(&htim15) and can test "TIM15_flag" for overflow
//    or, do GET_TIME_TMR15(count) and count == UINT32_MAX if timer overflowed.

extern volatile bool TIM15_flag;
extern volatile bool TIM16_flag;
extern volatile bool TIM17_flag;
extern volatile uint32_t TIM17_rollover_count;

// very small chance of a logic race in the following
#define RESET_TMR15_AND_FLAG {TIM15_flag = false; __HAL_TIM_SetCounter(&htim15,0);}
// The following is a more proper way of stopping the timer, clearing etc. May not work for 32 bit timers.
#define TMR15_OFF_RESET_CLR_FLAG {__HAL_TIM_DISABLE(&htim15);\
								  TIM15_flag = false; \
								  __HAL_TIM_SetCounter(&htim15,0);\
								  __HAL_TIM_ENABLE(&htim15);}
#define RESET_TMR16_AND_FLAG {TIM16_flag = false; __HAL_TIM_SetCounter(&htim16,0);}
#define RESET_TMR17_AND_FLAG {TIM17_flag = false; __HAL_TIM_SetCounter(&htim17,0);}

#define GET_TIME_TMR15(x) {x = __HAL_TIM_GetCounter(&htim15);\
						   if (TIM15_flag) { \
							   x = UINT32_MAX; \
						   }}

#define GET_TIME_TMR16(x) {x = __HAL_TIM_GetCounter(&htim16);\
						   if (TIM16_flag) { \
							   x = UINT32_MAX; \
						   }}

#define GET_TIME_TMR17(x) {x = __HAL_TIM_GetCounter(&htim17);\
						   if (TIM17_flag) { \
							   x = UINT32_MAX; \
						   }}

///////////////////////////////////////////////////////////////////////

// functions

void GPIO_Init(void);
void Error_Handler(const char * func, uint32_t line, const char * file);
bool SystemClock_Config(void);
bool TIM15_Init(void);
bool TIM15_Init_With_Period_Count(uint32_t period_count);
bool TIM16_Init(void);
bool TIM16_Init_With_Period_Count(uint32_t period_count);
bool TIM17_Init(void);
bool TIM17_Init_With_Period_Count(uint32_t period_count);
void waitForDebouncedPressAndReleasePU(uint32_t debounce_count, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint32_t getTMR17rolloverCountAndReset(void);
uint32_t getTMR17rolloverCount(void);

#endif
