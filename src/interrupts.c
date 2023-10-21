#include <stdbool.h>
#include "interrupts.h"
#include "hardware_profile.h"

// the most significant 32bit value will never overflow
static volatile uint32_t time_ms_MS = 0, time_ms_LS = 0;

volatile bool blueButtonPressed = false;
volatile bool U6interruptSemaphore = false;
volatile bool U7interruptSemaphore = false;
volatile bool lightInterruptSemaphore = false;

void NMI_Handler(void) {
#ifdef DEBUG_AND_TESTS
	HAL_GPIO_WritePin(TEST2_OUTPUT_GPIO_Port, TEST2_OUTPUT_Pin, GPIO_PIN_SET);
#endif
	while (true) {
	}
}

void HardFault_Handler(void) {
#ifdef DEBUG_AND_TESTS
	HAL_GPIO_WritePin(TEST2_OUTPUT_GPIO_Port, TEST2_OUTPUT_Pin, GPIO_PIN_SET);
#endif
	errorcode = HARD_FAULT;
	Error_Handler();
}

void SVC_Handler(void) {
#ifdef DEBUG_AND_TESTS
	HAL_GPIO_WritePin(TEST2_OUTPUT_GPIO_Port, TEST2_OUTPUT_Pin, GPIO_PIN_SET);
#endif
	while (true){
	}
}

void PendSV_Handler(void) {
#ifdef DEBUG_AND_TESTS
	HAL_GPIO_WritePin(TEST2_OUTPUT_GPIO_Port, TEST2_OUTPUT_Pin, GPIO_PIN_SET);
#endif
	while (true){
	}
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

// this gives an int64 value which is never repeated - used by BME680 BSEC
int64_t get_timestamp_us(void)
{
    static uint64_t last_time_us = 0;

    uint64_t t_us = get_time_us();

    if (t_us == last_time_us) {
    	t_us = t_us + (UINT64_C(1)); // to prevent a duplicated value
    }
    last_time_us = t_us;
	int64_t system_current_time = (int64_t) t_us; // this cast is safe for the next few thousand years
    return system_current_time;
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

void TIM16_IRQHandler(void) {
	TIM16_flag = true;
	HAL_TIM_IRQHandler(&htim16);
}

void TIM17_IRQHandler(void) {
	TIM17_flag = true;
	TIM17_rollover_count++;
	HAL_TIM_IRQHandler(&htim17);
}

// returns true only if pin is asserted on all Ndebounce consecutive reads.
bool debounceInterrupt(uint32_t Ndebounce, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinStateAsserted) {
	for (uint32_t i=0; i<Ndebounce; i++) {
		if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != PinStateAsserted) {
			return false;
		}
	}
	return true;
}

// IMPORTANT: MCU was stuck repeatedly servicing this interrupt because not all of
// the possible interrupt flags were being cleared in it, so it was immediately
// reentering due to a spurious interrupt source (NB: unused int sources should
// be disabled anyway)
// This function handles EXTI line 4 to 15 interrupts.
// ie. GPIO interrupts on any pin names Px4 to Px15. Can not have e.g. PB5 and PC5 interrupts as they share the same line.
void EXTI4_15_IRQHandler(void) {

	// check which pin is interrupting:
#ifdef ENABLE_LIGHT_SENSOR_INTERRUPTS
	if (__HAL_GPIO_EXTI_GET_FALLING_IT(LIGHT_INT_INTERNAL_Pin)) {
		__HAL_GPIO_EXTI_CLEAR_IT(LIGHT_INT_INTERNAL_Pin); // clear rising and falling
		// found that >=3 consecutive reads rejects mains switching noise. Thus use ~5 reads.
		if (debounceInterrupt(LIGHT_SENSOR_INTERRUPT_DEBOUNCE_POLLS, LIGHT_INT_INTERNAL_Port,
							  LIGHT_INT_INTERNAL_Pin, GPIO_PIN_RESET)) {
			lightInterruptSemaphore = true;
			// forward the light sensor interrupt to the external pin:
			assertExternalLightInterrupt();
		}
	}
#endif
#ifdef DEBUG_AND_TESTS
	if (__HAL_GPIO_EXTI_GET_IT(BLUE_BUTTON_Pin)) { // must define ENABLE_BLUE_BUTTON_INT for this to work.
		blueButtonPressed = true;
		__HAL_GPIO_EXTI_CLEAR_IT(BLUE_BUTTON_Pin); // clear rising and falling
	}
#endif
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |GPIO_PIN_7 |GPIO_PIN_8 |
				GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
}

// line 0/1 interrupt handler - for reset interrupt signal
// this handler is disabled during I2C2 transactions
void EXTI0_1_IRQHandler(void) {
	// check which pin is interrupting:
#ifdef ENABLE_RESET_INTERRUPT_LINE
	if (__HAL_GPIO_EXTI_GET_FALLING_IT(RESET_INT_Pin)) {
		__HAL_GPIO_EXTI_CLEAR_IT(RESET_INT_Pin); // clear rising and falling
		// found that >= 3 consecutive reads rejects mains switching noise. Thus use ~5 reads.
		if (debounceInterrupt(RESET_INTERRUPT_DEBOUNCE_POLLS, RESET_INT_GPIO_Port,
							RESET_INT_Pin, GPIO_PIN_RESET)) {
			// do anything here that needs doing before reset.
			deassertDataValid();
			NVIC_SystemReset();
		}
	}
#endif
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0 | GPIO_PIN_1);
}

