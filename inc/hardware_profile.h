#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "project_config.h"
#include <stdbool.h>

// error codes:
extern volatile uint32_t errorcode;
#define HARD_FAULT 999

///////////////////////////////////////////////////////////////////////

// interrupt priorities:
// priority must be number 0-3; , M0+ does not use subpriorities; IRQ number breaks tie
// Equal priority interrupts do not interrupt each other. Lower priorities interrupt higher ones.
// If two equal-priority interrupts are pending, the IRQn breaks the tie.

// systick is priority 0 with IRQn = -1
#define RESET_INT_IRQ_PRIORITY 0 // IRQn = 5
#define I2C1_IRQ_PRIORITY 1      // IRQn = 23
#define DMA_IRQ_PRIORITY 2       // IRQn = 9

// timers for debug process timing only:
#define TMR15_IRQ_PRIORITY 2 // IRQn = 20
#define TMR16_IRQ_PRIORITY 2 // IRQn = 21
#define TMR17_IRQ_PRIORITY 2 // IRQn = 22

#define BLUE_BUTTON_IRQ_PRIORITY 3 // has IRQn = 7  --- NOT USED

// note: the following 3 interrupts may not be enabled (may choose to use polling instead)
//       AND U7,U8 are for prototype system only.
#define LIGHT_U6_INT_INTERNAL_IRQ_PRIORITY 3 // IRQn = 7
#define LIGHT_U7_INT_INTERNAL_IRQ_PRIORITY 3 // IRQn = 7
#define LIGHT_U8_INT_INTERNAL_IRQ_PRIORITY 3 // IRQn = 7

#define TMR14_IRQ_PRIORITY 3   // used for PPD42. IRQn = 19
#define TMR3_IRQ_PRIORITY 3    // used for microphone warmup time. IRQn = 16

//#define I2C2_IRQ_PRIORITY // not used. IRQn = 24

///////////////////////////////////////////////////////////////////////

// defines

#define AHB_CLK_DIV RCC_SYSCLK_DIV1
#define APB1_CLK_DIV RCC_HCLK_DIV1

#define PASSFAIL_STR(x) (x ? "pass" : "fail")

//////////////////////////////////////////////////////////////////////////////////

// UART:

#define USE_USART4 // choose module to use here, assuming HAL_UART_MODULE_ENABLED is defined

// options for specific UART modules:
#define LPUART1_TX_PIN GPIO_PIN_2 // these pins were those used by the STM32G071 nucleo board before snapping it.
#define LPUART1_TX_PORT GPIOA
#define LPUART1_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // must agree with LPUART1_TX_PORT

#define LPUART1_RX_PIN GPIO_PIN_3
#define LPUART1_RX_PORT GPIOA
#define LPUART1_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define USART4_TX_PIN GPIO_PIN_0
#define USART4_TX_PORT GPIOA
#define USART4_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // must agree with USART4_TX_PORT

#define USART4_RX_PIN GPIO_PIN_1
#define USART4_RX_PORT GPIOA
#define USART4_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

// generic options:
#define UART_BAUD 115200
#define UART_WORDLENGTH UART_WORDLENGTH_8B
#define UART_STOPBITS UART_STOPBITS_1
#define UART_PARITY UART_PARITY_NONE

//////////////////////////////////////////////////////////////////////////////////

#ifdef DEBUG_AND_TESTS
	#define ERROR_LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
	#define ERROR_LED_Pin GPIO_PIN_6
	#define ERROR_LED_GPIO_Port GPIOA

	/*
	// if using the nucleo board with built-in blue button:
	#define BLUE_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
	#define BLUE_BUTTON_PULL GPIO_NOPULL
	#define BLUE_BUTTON_Pin GPIO_PIN_13
	#define BLUE_BUTTON_GPIO_Port GPIOC
	#define BLUE_BUTTON_IRQn EXTI4_15_IRQn
	*/
	// if using the prototype MS1 board (no PC13 GPIO so use PA7)
	#define BLUE_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
	#define BLUE_BUTTON_PULL GPIO_PULLUP
	#define BLUE_BUTTON_Pin GPIO_PIN_7
	#define BLUE_BUTTON_GPIO_Port GPIOA
	#define BLUE_BUTTON_IRQn EXTI4_15_IRQn

	#define TEST1_OUTPUT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
	#define TEST1_OUTPUT_Pin GPIO_PIN_8
	#define TEST1_OUTPUT_GPIO_Port GPIOA

	#define TEST2_OUTPUT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
	#define TEST2_OUTPUT_Pin GPIO_PIN_2
	#define TEST2_OUTPUT_GPIO_Port GPIOB

	// interrupt lines for extra light sensor chips:
	#define LIGHT_U7_INT_INTERNAL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
	#define LIGHT_U7_INT_INTERNAL_Pin GPIO_PIN_6
	#define LIGHT_U7_INT_INTERNAL_Port GPIOC
	#define LIGHT_U7_INT_INTERNAL_IRQn EXTI4_15_IRQn

	#define LIGHT_U8_INT_INTERNAL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
	#define LIGHT_U8_INT_INTERNAL_Pin GPIO_PIN_9
	#define LIGHT_U8_INT_INTERNAL_Port GPIOA
	#define LIGHT_U8_INT_INTERNAL_IRQn EXTI4_15_IRQn
#endif

#define BUSY_OUTPUT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUSY_OUTPUT_Pin GPIO_PIN_1
#define BUSY_OUTPUT_GPIO_Port GPIOB

#define RESET_INT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define RESET_INT_Pin GPIO_PIN_0
#define RESET_INT_GPIO_Port GPIOB
#define RESET_INT_IRQn EXTI0_1_IRQn

// Spurious interrupts: found that >=3 consecutive pin reads in the ISR rejects mains switching noise. Thus use ~5 reads.
#define LIGHT_SENSOR_INTERRUPT_DEBOUNCE_POLLS 5

// Internal light sensor interrupt (nb: also U7, U8 for prototype system):
//#define ENABLE_LIGHT_SENSOR_INTERRUPTS -> moved to project_config
#define LIGHT_U6_INT_INTERNAL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define LIGHT_U6_INT_INTERNAL_Pin GPIO_PIN_10
#define LIGHT_U6_INT_INTERNAL_Port GPIOA
#define LIGHT_U6_INT_INTERNAL_IRQn EXTI4_15_IRQn

// sound interrupt external output (amplitude over threshold output) - to be open drain, no pull
#define SOUND_INT_EXTERNAL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define SOUND_INT_EXTERNAL_Pin GPIO_PIN_4
#define SOUND_INT_EXTERNAL_Port GPIOB

// light interrupt external output (forwarding the light sensor interrupt) - to be open drain, no pull
#define LIGHT_INT_EXTERNAL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define LIGHT_INT_EXTERNAL_Pin GPIO_PIN_5
#define LIGHT_INT_EXTERNAL_Port GPIOB

// data valid output - to be open drain, no pull
#define READY_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define READY_Pin GPIO_PIN_3
#define READY_Port GPIOB

// I2C address select pin:
#define ADDR_SELECT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define ADDR_SELECT_Pin GPIO_PIN_9
#define ADDR_SELECT_Port GPIOB

///////////////////////////////////////////////////////////////////////

extern DMA_HandleTypeDef hdma_spi1_rx;

// I2S1 pins

// enable an internal pulldown on the SD line so it does not float when the right audio channel is enabled
// this removes the need for an external pulldown.
#define I2S1_SD_INTERNAL_PD // if not defined, pin has no PU or PD (the default setting from cubeMX)

#define I2S1_CLK_PIN GPIO_PIN_5
#define I2S1_CLK_PORT GPIOA
#define I2S1_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // must agree with I2S1_CLK_PORT
#define I2S1_CLK_AF_MAP GPIO_AF0_SPI1 // need to find this with cubeMX

#define I2S1_WS_PIN GPIO_PIN_4
#define I2S1_WS_PORT GPIOA
#define I2S1_WS_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S1_WS_AF_MAP GPIO_AF0_SPI1

#define I2S1_SD_PIN GPIO_PIN_2
#define I2S1_SD_PORT GPIOA
#define I2S1_SD_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S1_SD_AF_MAP GPIO_AF0_SPI1

///////////////////////////////////////////////////////////////////////

// I2C ports

#define I2C1_SDA_PIN GPIO_PIN_7
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() // must agree with I2C1_SDA_PORT

#define I2C1_SCL_PIN GPIO_PIN_8
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2C2_SDA_PIN GPIO_PIN_12
#define I2C2_SDA_PORT GPIOA
#define I2C2_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define I2C2_SCL_PIN GPIO_PIN_11
#define I2C2_SCL_PORT GPIOA
#define I2C2_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

///////////////////////////////////////////////////////////////////////

// TMR2 gating for PPD42 readout

#define TIM2_GATE_PIN GPIO_PIN_15
#define TIM2_GATE_PORT GPIOA
#define TIM2_GATE_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // must agree with TIM2_GATE_PORT

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
void init_I2C2_pins_manual(void);
void deinit_I2C2_pins_manual(void);
void Error_Handler(void);
bool SystemClock_Config(void);
bool TIM15_Init(void);
bool TIM15_Init_With_Period_Count(uint32_t period_count);
bool TIM16_Init(void);
bool TIM16_Init_With_Period_Count(uint32_t period_count);
bool TIM17_Init(void);
bool TIM17_Init_With_Period_Count(uint32_t period_count);
void waitForDebouncedPressAndReleasePU(uint32_t debounce_count, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void assertExternalSoundInterrupt(void);
void deassertExternalSoundInterrupt(void);
void assertDataValid(void);
void deassertDataValid(void);
void deassertExternalLightInterrupt(void);
void assertExternalLightInterrupt(void);
void assertBusy(void);
void deassertBusy(void);
uint32_t getTMR17rolloverCountAndReset(void);
uint32_t getTMR17rolloverCount(void);

#endif
