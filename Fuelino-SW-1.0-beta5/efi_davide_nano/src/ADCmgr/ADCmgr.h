// Davide Cavaliere
// www.monocilindro.com
// dadez87-at-gmail-com
// ADC aquisition library for Atmel ATmega 328p
// Date: 28 February 2017
// This library is used, on Fuelino project, to acquire data from sensors (Throttle, Lambda) continuously using ADC interrupt

#ifndef ADCmgr_h
#define ADCmgr_h

#include <Arduino.h>
#define ADCMGR_CYCLE_TIME_MEASURE 0 // Defines if needed to measure the cycle time ("ADCMGR_PINS_ORDER_SIZE" measures)
#define ADCMGR_DI1_PIN 0 // Digital Input DI1 (A0)
#define ADCMGR_THROTTLE_PIN 2 // Throttle Pin (A2)
#define ADCMGR_LAMBDA_PIN 3 // Lambda Pin (A3)
#define ADCMGR_VBATTERY_PIN 6 // Voltage Battery Pin (A6)
#define ADCMGR_ENABLE_OR_INT_PIN 7 // ENABLE_OR_INT (A6)
#define ADCMGR_TEMPERATURE_PIN 8 // Internal Temperature
#define ADCMGR_PINS_ORDER_SIZE 5 // Number of pins to be read continuously (max should be 8!!!)
#define ADCMGR_LAMBDA_ACQ_BUF_HEAD 9 // Header size (0x4C, Engine info 4x2 bytes)
#define ADCMGR_LAMBDA_ACQ_BUF_SIZE 32 // Total acquisitions of Lambda Sensor voltage (single byte, 0 - 255)
#define ADCMGR_LAMBDA_ACQ_BUF_TAIL 4 // Tail size (acquisition time + delta_time_tick)
#define ADCMGR_LAMBDA_ACQ_BUF_TOT (ADCMGR_LAMBDA_ACQ_BUF_HEAD + ADCMGR_LAMBDA_ACQ_BUF_SIZE + ADCMGR_LAMBDA_ACQ_BUF_TAIL + 2) // Total size of packet: header, data, tail, checksum

// Indexes of arrays "ADCmgr_pins_order[]" and "ADCmgr_measures[ADCMGR_PINS_ORDER_SIZE]" in which the info is stored
#define ADCMGR_THROTTLE_INDEX 1 // Index to read Throttle signal from ADC module
#define ADCMGR_LAMBDA_INDEX 2 // Index to read Lambda signal from ADC module
#define ADCMGR_BATTERY_INDEX 3 // Index to read Battery signal from ADC module

// Variables
//extern const uint8_t ADCmgr_pins_order[]; // Contains the physical number of the Analog pin to be read (example: [A]1, [A]2, ..., [A]7)
//extern volatile uint8_t ADCmgr_pins_buffer_busy; // Buffer busy status. This flag is set when reading the status. 8 bits.
//extern volatile uint16_t ADCmgr_measures[]; // Contains the value read
//extern volatile uint8_t ADCmgr_meas_binary; // Digital value (0 .. 1) 8 bits

// For debugging purpose (time calculation of the complete reading time for one cycle)
#if ADCMGR_CYCLE_TIME_MEASURE // measure the cycle time
extern volatile uint16_t ADCmgr_read_time_delta_buf; // time required by one ADC complete measurement cycle
extern volatile bool ADCmgr_read_time_buf_busy; // becomes "true" when Main Loop is reading the buffer
#endif

// For Lambda sensor buffer acquisition
extern volatile uint8_t ADCmgr_lambda_acq_buf[]; // Lambda sensor signal values buffer
extern volatile bool ADCmgr_lambda_acq_buf_filled; // Becomes true after the buffer is completely filled
extern volatile uint8_t ADCmgr_lambda_acq_prescaler_max; // Prescaler to reduce the acquisition frequency of Lambda signal (max value)

// Functions
extern void ADCmgr_init(); // Initialization
extern uint16_t ADCmgr_read_pin_now(uint8_t ADC_pin); // Read pin voltage, waits until the ADC process has finished, then returns the value
//extern void ADCmgr_program_pin_read(uint8_t ADC_pin); // Read pin voltage, by starting the ADC process. The reading will be done during interrupt event

// For reading Throttle and Lambda and Battery Status signals
extern uint16_t ADCmgr_throttle_signal_read();
extern uint16_t ADCmgr_lambda_signal_read();
extern uint8_t ADCmgr_battery_status_read();
extern uint8_t ADCmgr_binary_inputs_status_read();

#endif