#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#define HAL_I2S_MODULE_ENABLED

// Define this to run tests
//#define TESTS

// Define this to print debugging messages over UART
//#define DEBUG_PRINT

// Define to use the nucleo board LED and pushbutton
//#define NUCLEO_BOARD
// Use interrupt rather than poll?
#define ENABLE_BLUE_BUTTON_INT

// clock setup
#define SYSCLK_FREQ_HZ 32000000

/////////////////////////////////////////////////////////////////////////////

// Sound settings

#define FFT_N 128
#define I2S_AUDIOFREQ I2S_AUDIOFREQ_16K // can be 16, 32, 48

//#define FILTER_SPL // if defined, SPL is averaged over N readings, then SPL calc stops.
				   // if not defined, SPL is continuously calculated on each DMA interrupt
				   // and can be read at any time.
#define FILTER_SPL_N 20 // how many consecutive SPL calculations to average over
						// NOTE: this is NOT a moving average: Accumulate N readings
						// and average, then start again.

//////////////////////////////////////////////////////////////////////////////////

#endif

