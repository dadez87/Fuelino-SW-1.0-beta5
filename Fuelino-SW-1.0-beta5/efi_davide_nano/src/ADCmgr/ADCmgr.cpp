// Davide Cavaliere
// www.monocilindro.com
// dadez87-at-gmail-com
// ADC aquisition library for Atmel ATmega 328p
// Date: 27 February 2017
// This library is used, on Fuelino project, to acquire data from sensors (Throttle, Lambda) continuously using ADC interrupt

#ifndef ADCmgr_cpp
#define ADCmgr_cpp

#include "ADCmgr.h"
#include "../INJmgr/INJmgr.h" // INJ manager. To have access to Timer0 reading

// Variables needed for pin Analog Voltage acquisition
volatile uint8_t ADCmgr_pin_read_now_index = 0; // Start from the first element of the array
const uint8_t ADCmgr_pins_order[] = {ADCMGR_DI1_PIN, ADCMGR_THROTTLE_PIN, ADCMGR_LAMBDA_PIN, ADCMGR_VBATTERY_PIN, ADCMGR_ENABLE_OR_INT_PIN, ADCMGR_TEMPERATURE_PIN};
volatile uint8_t ADCmgr_pins_buffer_busy = 0; // buffer busy status. This flag is set when reading the status. 8 bits (1 bit per signal)
volatile uint16_t ADCmgr_measures[ADCMGR_PINS_ORDER_SIZE]; // Analog value (0 .. 1023)
volatile uint8_t ADCmgr_meas_binary = 0; // Digital value (0 .. 1) 8 bits (1 bit per signal)

// For debugging purpose (calculation of cycle time)
#if ADCMGR_CYCLE_TIME_MEASURE // measure the cycle time
volatile uint16_t ADCmgr_read_time_end = 0;
volatile uint16_t ADCmgr_read_time_delta_buf = 0; // time required by one ADC complete measurement cycle
volatile bool ADCmgr_read_time_buf_busy = false; // flag set by Main Loop to lock the access to the buffer value, to avoid 2 bytes data corruption
#endif

// For Lambda sensor buffer acquisition
volatile uint8_t ADCmgr_lambda_acq_buf[ADCMGR_LAMBDA_ACQ_BUF_TOT]; // Lambda sensor signal values buffer
volatile uint8_t ADCmgr_lambda_acq_buf_index = ADCMGR_LAMBDA_ACQ_BUF_HEAD; // Index when storing Lambda sensor voltage on buffer
volatile bool ADCmgr_lambda_acq_buf_filled = false; // Becomes true after the buffer is completely filled
volatile uint16_t ADCmgr_lambda_acq_buf_time_start; // Time when first buffer sample is acquired
volatile uint8_t ADCmgr_lambda_acq_prescaler_max = 0; // Prescaler to reduce the acquisition frequency of Lambda signal (max value)
volatile uint8_t ADCmgr_lambda_acq_prescaler_cnt = 0; // Prescaler to reduce the acquisition frequency of Lambda signal (counter value)


// Reads the ADC pin status, and returns the ADC value. Each conversion requires about 25 clock cycles (at 125kHz).
uint16_t ADCmgr_read_pin_now(uint8_t ADC_pin){

	ADMUX &= 0xF0; // Removes previous pin setting
	ADMUX |= (ADC_pin & 0x0F); // Pin number setting
	
	ADCSRA |= (1<<ADEN); // Enables the ADC - Bit 7 ? ADEN: ADC Enable
	ADCSRA &= ~(1<<ADIE); // Disables interrupt - Bit 3 ? ADIE: ADC Interrupt Enable
	ADCSRA |= (1<<ADSC); // Start converting

	while ((ADCSRA & _BV(ADSC))); // Wait until the end of the conversion

	// Read ADC measurement
	uint8_t adc_L = ADCL; // low part
	uint8_t adc_H = ADCH; // high part
	
	ADCSRA &= ~(1<<ADEN); // Disables the ADC - Bit 7 ? ADEN: ADC Enable
	
	uint16_t adc_value = ((uint16_t)adc_H << 8) | (uint16_t)adc_L; // Calculates the ADC value
	
	return adc_value;
}


// Programs the reading of an ADC pin. Reading must be done using the interrupt. Each conversion requires about 25 clock cycles (at 125kHz).
void ADCmgr_program_pin_read(uint8_t ADC_pin){

	ADMUX &= 0xF0; // Removes previous pin setting
	ADMUX |= (ADC_pin & 0x0F); // Pin number setting

	ADCSRA |= (1<<ADEN); // Enables the ADC - Bit 7 ? ADEN: ADC Enable
	ADCSRA |= (1<<ADIE); // Enables interrupt - Bit 3 ? ADIE: ADC Interrupt Enable
	ADCSRA |= (1<<ADSC); // Start converting

}


// Initializes the ADC
void ADCmgr_init(){
	
	pinMode(A0, INPUT_PULLUP); // Input pullup on DI1 pin (A0)
	
	// ADMUX - ADC Multiplexer Selection Register 
	ADMUX = 0b01000000; // AVcc (5V) as reference, ADLAR = 0 (ADLAR: ADC Left Adjust Result)
	
	// ADCSRA - ADC Control and Status Register A
	ADCSRA = ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)); //Prescaler at 128 so we have an 125Khz clock source, Bits 2:0 ? ADPS[2:0]: ADC Prescaler Select Bits
	// Bit 3 ? ADIE: ADC Interrupt Enable
	// Bit 4 ? ADIF: ADC Interrupt Flag
	// Bit 5 ? ADATE: ADC Auto Trigger Enable
	// Bit 6 ? ADSC: ADC Start Conversion
	// Bit 7 ? ADEN: ADC Enable
	
	// ADCSRB - ADC Control and Status Register B
	ADCSRB = 0;
	
	ADCmgr_lambda_acq_buf[0] = 0x4C; // Lambda packet identifier ('L')
	
	ADCmgr_program_pin_read(ADCmgr_pins_order[0]); // Starts the ADC conversion (which will then continue indefinitely)
}


// Acquires throttle voltage signal from ADC module
uint16_t ADCmgr_throttle_signal_read(){
	ADCmgr_pins_buffer_busy |=  (1<<ADCMGR_THROTTLE_INDEX); // Locks buffer write access
	uint16_t throttle_tmp_rd = ADCmgr_measures[ADCMGR_THROTTLE_INDEX]; // Reads throttle voltage from ADCmgr software module
	ADCmgr_pins_buffer_busy &=  ~(1<<ADCMGR_THROTTLE_INDEX); // Unlocks buffer write access
	return throttle_tmp_rd;
}


// Acquires lambda voltage signal from ADC module
uint16_t ADCmgr_lambda_signal_read(){
	ADCmgr_pins_buffer_busy |=  (1<<ADCMGR_LAMBDA_INDEX); // Locks buffer write access
	uint16_t lambda_tmp_rd = ADCmgr_measures[ADCMGR_LAMBDA_INDEX]; // update buffer
	ADCmgr_pins_buffer_busy &=  ~(1<<ADCMGR_LAMBDA_INDEX); // Unlocks buffer write access
	return lambda_tmp_rd;
}


// Acquires input battery voltage status
uint8_t ADCmgr_battery_status_read(){
	return ((ADCmgr_meas_binary & (1 << ADCMGR_BATTERY_INDEX)) >> ADCMGR_BATTERY_INDEX);
}


// Reads the raw status of digitalized signals
uint8_t ADCmgr_binary_inputs_status_read(){
	return ADCmgr_meas_binary;
}


// Interrupt service routine for the ADC completion
ISR(ADC_vect){

	// Read ADC measurement
	uint8_t adc_L = ADCL; // low part
	uint8_t adc_H = ADCH; // high part
	ADCSRA &= ~(1<<ADEN); // Disables the ADC - Bit 7 - ADEN: ADC Enable
	if (!(ADCmgr_pins_buffer_busy & (1 << ADCmgr_pin_read_now_index))) ADCmgr_measures[ADCmgr_pin_read_now_index] = ((uint16_t)adc_H << 8) | (uint16_t)adc_L; // Calculates the ADC value (2 bytes) and stores it into the buffer, if not busy
	if (adc_H == 0) {
		ADCmgr_meas_binary &= ~(1 << ADCmgr_pin_read_now_index); // OFF, if <= 255
	}else{
		ADCmgr_meas_binary |= (1 << ADCmgr_pin_read_now_index); // ON, if >= 256
	}

	// Check if there is anything special to do with the present measured signal
	switch (ADCmgr_pins_order[ADCmgr_pin_read_now_index]){
		case ADCMGR_LAMBDA_PIN: // Lambda sensor signal acquired
			if ((ADCmgr_lambda_acq_prescaler_max) && (ADCmgr_lambda_acq_buf_filled == false)){ // Filling request coming from external module, and the buffer is not yet filled completely
				ADCmgr_lambda_acq_prescaler_cnt++; // divider
				if (ADCmgr_lambda_acq_prescaler_cnt >= ADCmgr_lambda_acq_prescaler_max){ // divider
					ADCmgr_lambda_acq_prescaler_cnt = 0; // divider
					if (ADCmgr_lambda_acq_buf_index == ADCMGR_LAMBDA_ACQ_BUF_HEAD) ADCmgr_lambda_acq_buf_time_start = INJmgr.Timer0_tick_counts(); // First element of the buffer. 1 tick = 4us
					if (adc_H != 0) { // Over 1.25V
						ADCmgr_lambda_acq_buf[ADCmgr_lambda_acq_buf_index] = 0xFF; // Saturation to max value, about 1.25V
					}else{
						ADCmgr_lambda_acq_buf[ADCmgr_lambda_acq_buf_index] = adc_L; // Save low byte (0 - 255), 255 corresponds to a max value of about 1.25V
					}
					ADCmgr_lambda_acq_buf_index++; // increase counter
					if (ADCmgr_lambda_acq_buf_index >= (ADCMGR_LAMBDA_ACQ_BUF_HEAD+ADCMGR_LAMBDA_ACQ_BUF_SIZE)){ // the Lambda buffer has been completely filled
						uint16_t ADCmgr_lambda_acq_buf_time_delta_buf = INJmgr.Timer0_tick_counts() - ADCmgr_lambda_acq_buf_time_start; // Total acquisition period between first and last sample
						ADCmgr_lambda_acq_buf[(ADCMGR_LAMBDA_ACQ_BUF_HEAD + ADCMGR_LAMBDA_ACQ_BUF_SIZE)] = (uint8_t)(ADCmgr_lambda_acq_buf_time_delta_buf & 0xff); // LSB
						ADCmgr_lambda_acq_buf[(ADCMGR_LAMBDA_ACQ_BUF_HEAD + ADCMGR_LAMBDA_ACQ_BUF_SIZE) + 1] = (uint8_t)((ADCmgr_lambda_acq_buf_time_delta_buf >> 8) & 0xff); // MSB
						ADCmgr_lambda_acq_buf_index = ADCMGR_LAMBDA_ACQ_BUF_HEAD; // Set the counter to first value, for next use
						ADCmgr_lambda_acq_prescaler_max = 0; // Filling request turned OFF (need a new request to start filling again)
						ADCmgr_lambda_acq_buf_filled = true; // Indicates that the buffer has been completely filled, and is now ready to use
					}
				}
			}else{ // no need to fill the buffer
				ADCmgr_lambda_acq_buf_index = ADCMGR_LAMBDA_ACQ_BUF_HEAD; // Set the counter to first value, just to make sure
				ADCmgr_lambda_acq_prescaler_cnt = 0; // prescaler counter initialized to 0
			}
			break;
		  
		default:
			break;
	  
	}

	// Programs the next ADC reading
	ADCmgr_pin_read_now_index++; // Increase pin
	if (ADCmgr_pin_read_now_index == ADCMGR_PINS_ORDER_SIZE) { // Read all pins
		ADCmgr_pin_read_now_index = 0; // restart from the first pin
		// Cycle time calculation
		#if ADCMGR_CYCLE_TIME_MEASURE // measure the cycle time
		uint16_t ADCmgr_read_time_start = ADCmgr_read_time_end; // starting time is the time at which it ended last measurement cycle
		ADCmgr_read_time_end = INJmgr.Timer0_tick_counts(); // 1 tick = 4us
		if (ADCmgr_read_time_buf_busy == false){ // if the buffer is not being used, update the buffer
			ADCmgr_read_time_delta_buf = ADCmgr_read_time_end - ADCmgr_read_time_start; // time required for one complete cycle of acquisitions
		}
		#endif
	}
	ADCmgr_program_pin_read(ADCmgr_pins_order[ADCmgr_pin_read_now_index]); // programs the next ADC read

}

#endif