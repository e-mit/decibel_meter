#include <stdbool.h>
#include "interrupts.h"
#include "hardware_profile.h"

void NMI_Handler(void) {
	errorHandler(__func__, __LINE__, __FILE__);
}

void HardFault_Handler(void) {
	errorHandler(__func__, __LINE__, __FILE__);
}

void SVC_Handler(void) {
	errorHandler(__func__, __LINE__, __FILE__);
}

void PendSV_Handler(void) {
	errorHandler(__func__, __LINE__, __FILE__);
}

void SysTick_Handler(void) {
	HAL_IncTick();
}

