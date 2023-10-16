#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// modules required:
#define HAL_I2S_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED

// define this to enable serial port printing BUT does not by itself cause any printing
//#define HAL_UART_MODULE_ENABLED
//HAL_USART_MODULE_ENABLED

// define this to do 2 separate things: print various info over serial port AND enable some debugging functions
// must have also defined HAL_UART_MODULE_ENABLED
//#define DEBUG_AND_TESTS  // increases project size by approx 15kB.

// clock setup
#define SYSCLK_FREQ_HZ 32000000


#define ENABLE_SLEEP_WAITING // sleep at end of main loop, otherwise do busy wait

// found that >=3 consecutive reads rejects mains switching noise. Thus use ~5 reads.
#define RESET_INTERRUPT_DEBOUNCE_POLLS 5

/////////////////////////////////////////////////////////////////////////////

#define SENSOR_INIT_ATTEMPTS 10

// sound
#define FFT_N 128
#define I2S_AUDIOFREQ I2S_AUDIOFREQ_16K // can be 16, 32, 48

#define FILTER_SPL // if defined, SPL is averaged over N readings, then SPL calc stops.
				   // if not defined, SPL is continuously calculated on each DMA interrupt
				   // and can be read at any time.
#define FILTER_SPL_N 20 // how many consecutive SPL calculations to average over
						// NOTE: this is NOT a moving average: Accumulate N readings
						// and average, then start again.

//////////////////////////////////////////////////////////////////////////////////

#endif

