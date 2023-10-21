#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32g0xx_hal.h"
#include <stdbool.h>

bool UART_Init(void);
void UARTprint(char * str, uint16_t len);

#endif
