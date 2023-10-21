#include <stdbool.h>
#include "interrupts.h"
#include "hardware_profile.h"

// The most significant 32bit value will never overflow
static volatile uint32_t time_ms_MS = 0, time_ms_LS = 0;

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

uint64_t get_time_us(void) {
	uint64_t ms = get_time_ms();
	return (ms*1000);
}

// use the general method for reading a non-atomic variable which may be
// changed by an interrupt at any time:
uint64_t get_time_ms(void) {
	/*
	Read the upper half of the timer into H.
	Read the lower half of the timer into L.
	Read the upper half of the timer again into H'.
	If H == H' then return {H, L}, otherwise go back to 1.
	*/
	volatile uint32_t t_ms_MS = 0, t_ms_LS = 0, t_ms_MS2 = 1;

	while (t_ms_MS != t_ms_MS2) {
		t_ms_MS = time_ms_MS;
		t_ms_LS = time_ms_LS;
		t_ms_MS2 = time_ms_MS;
	}

	return ((((uint64_t) t_ms_MS) << 32)|((uint64_t) t_ms_LS));
}

void SysTick_Handler(void) {
	HAL_IncTick();
	time_ms_LS++;
	if (time_ms_LS == 0) {
		time_ms_MS++;
	}
}

void TIM15_IRQHandler(void) {
	TIM15_flag = true;
	HAL_TIM_IRQHandler(&htim15);
}

