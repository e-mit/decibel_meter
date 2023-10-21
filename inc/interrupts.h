#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <stdint.h>
#include "stm32g0xx_hal.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint64_t get_time_ms(void);
uint64_t get_time_us(void);

void TIM15_IRQHandler(void);

#endif
