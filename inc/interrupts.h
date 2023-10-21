#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <stdint.h>
#include "stm32g0xx_hal.h"

extern volatile bool blueButtonPressed;
extern volatile bool U6interruptSemaphore;
extern volatile bool U7interruptSemaphore;
extern volatile bool lightInterruptSemaphore;

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint64_t get_time_ms(void);
uint64_t get_time_us(void);
int64_t get_timestamp_us(void);

void TIM15_IRQHandler(void);
void TIM16_IRQHandler(void);
void TIM17_IRQHandler(void);

void EXTI4_15_IRQHandler(void);
void EXTI0_1_IRQHandler(void);

bool debounceInterrupt(uint32_t Ndebounce, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinStateAsserted);

#endif
