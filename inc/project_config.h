#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// modules required:
#define HAL_I2C_MODULE_ENABLED
#define HAL_I2S_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED

// define this to enable serial port printing BUT does not by itself cause any printing
#define HAL_UART_MODULE_ENABLED

// define this to do 2 separate things: print various info over serial port AND enable some debugging functions
// must have also defined HAL_UART_MODULE_ENABLED
//#define DEBUG_AND_TESTS  // increases project size by approx 15kB.

// clock setup
#define SYSCLK_FREQ_HZ 32000000

/////////////////////////////////////////////////////////////////////////////

#define I2C1_DISABLE_TIMEOUT_MS 20 // time to wait for current transaction to end
// this is used when waiting for current transaction to end immediately prior to disabling I2C1

#define ENABLE_SLEEP_WAITING // sleep at end of main loop, otherwise do busy wait

/////////////////////////////////////////////////////////////////////////////

// external interrupt line to replace hardware reset via direct NRST interface
//#define ENABLE_RESET_INTERRUPT_LINE // define this to enable it

// found that >=3 consecutive reads rejects mains switching noise. Thus use ~5 reads.
#define RESET_INTERRUPT_DEBOUNCE_POLLS 5

/////////////////////////////////////////////////////////////////////////////

#define SENSOR_INIT_ATTEMPTS 10

/////////////////////////////////////////////////////////////////////////////

// I2C1 slave:

#define RECEIVE_BUFFER_SIZE 4 // this must be at least one greater than largest accepted write.
#define USE_I2C1_INTERRUPTS

/////////////////////////////////////////////////////////////////////////////

//// BME680:

#define BME680_OD_HEATER_TEMP 320
#define BME680_OD_HEATER_DUR 165 // manually adjusted to give a similar G_ohm value in OD mode as in BSEC modes:
// LP mode uses 197 ms, 320C
// ULP / ULP_PLUS uses 1943 ms, 400C
#define BSEC_CAL_PERIOD_4D // BSEC_CAL_PERIOD_28D or BSEC_CAL_PERIOD_4D
#define VOLTAGE_33V // VOLTAGE_33V or VOLTAGE_18V

#define ADUST_IAQ_DATA // if IAQ==0 (usu just after loading state) then set to default values

#define MODE_CHANGE_TIMEOUT_MS 20
// used for BME return to standby wait

/////////////////////
// BSEC state save to flash options:

// CANNOT USE FLASH SAVING: does not work with sound DMA; problem not fixed.
//#define BSEC_STATE_FLASH_OPS // if defined, attempt to save/load to/from flash, else do not
// if this is not defined, the following options are irrelevant.

// NOTE: for the saving to work, must have allocated 3 pages of flash at the end of memory in the .ld file.

#define MIN_BSEC_STATE_SAVE_ACCURACY 0 // only save state if acc >= this
// TODO: change the above to 1

// how long should pass before saving state
#define LP_STATE_SAVE_LOOPS       3
#define ULP_PLUS_STATE_SAVE_LOOPS 2 // NOTE: this is DATA OUTPUT loops (i.e. loop = 100 s)
#define ULP_STATE_SAVE_LOOPS      2
// NB: the bsec example saves state every ~8 hours; prefer to save every 1-2 hrs
// note that min flash endurance is 10,000 cycles -> 40,000 hours = 4.5 years

#define BME_TEMPERATURE_OFFSET 0.8F // this value is subtracted from the measured raw value

//////////////////////////////////////////////////////////////////////////////////

// cycle mode options

#define CYCLE_DV_DEASSERTION_TIME_MS 50 // fixed period on each cycle during
										// which DV is deasserted and I2C (potentially) disabled
#define DV_DEASSERT_I2C_MARGIN_MS 5 // out of CYCLE_DV_DEASSERTION_TIME_MS, how long is the I2C bus
									// allowed to continue operating for?

// time from the nominal start of the (data output) cycle to the start of the ready deassertion.
// the following have lots of margin for cmd processing etc
// NOTE: on the first cycle (immediately after entering cycle mode) ready will remain deasserted until the
// normal assertion point. Thus the time after entering the cycle (after the transition cmd is completed)
// up to the point of DV assertion is CYCLE_DV_DEASSERTION_TIME_MS + the relevant one of the below.
#define LP_LOOP_TIME_TO_DV_MS       500
#define ULP_PLUS_LOOP_TIME_TO_DV_MS 2500 // only applies to data-output cycles
#define ULP_LOOP_TIME_TO_DV_MS      2500
#define OD_LOOP_TIME_TO_DV_MS       500

//////////////////////////////////////////////////////////////////////////////////

// on-demand readout time, during which READY is high and I2C external is disabled
// AND during which the particle sensing is disabled.

#define OD_READOUT_PERIOD_MS 500

//////////////////////////////////////////////////////////////////////////////////

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

// no particle sensor settings are now needed, as have changed from the general-purpose
// but PPD42-only module PPD42.c/h to the new, specialised ParticleSensor.c/h

/////////////////////////////////////////////////////////////////////////////////////

// light:

// which sensor to use:
#define USE_VEML_6030   // USE_VEML_6035 or USE_VEML_6030

#ifdef USE_VEML_6030
// choose which light sensor to use as interrupt input:
#define LIGHT_INT_INTERNAL_Port LIGHT_U6_INT_INTERNAL_Port
#define LIGHT_INT_INTERNAL_Pin  LIGHT_U6_INT_INTERNAL_Pin
#define LIGHT_INT_INTERNAL_IRQn LIGHT_U6_INT_INTERNAL_IRQn

#define VEML_I2CADDR_7BIT VEML6030_I2CADDR_7BIT

#elif defined USE_VEML_6035

#define LIGHT_INT_INTERNAL_Port LIGHT_U7_INT_INTERNAL_Port
#define LIGHT_INT_INTERNAL_Pin  LIGHT_U7_INT_INTERNAL_Pin
#define LIGHT_INT_INTERNAL_IRQn LIGHT_U7_INT_INTERNAL_IRQn

#define VEML_I2CADDR_7BIT VEML6035_I2CADDR_7BIT

#endif

// choose whether to use MCU interrupt or polling to forward the light interrupt:
#define ENABLE_LIGHT_SENSOR_INTERRUPTS // else uses polling

//////////////////////////////////////////////////////////////////////////////////////

#endif

