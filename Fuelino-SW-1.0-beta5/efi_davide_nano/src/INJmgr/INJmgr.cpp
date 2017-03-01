#ifndef INJmgr_cpp
#define INJmgr_cpp

#include "INJmgr.h"
#include "Tempo/Tempo.h"
#include "../ADCmgr/ADCmgr.h" // ADC manager. To have access to Analog readings

// GLOBAL VARIABLES

volatile uint16_t injection_counter=0; // counts the combustion cycles
volatile uint16_t inj_start_tick_last=0; // time at which injection was started last time
volatile uint16_t delta_time_tick=0; // distanza tra due iniezioni (2rpm)
volatile uint16_t delta_inj_tick=0; // tempo di iniezione input
volatile uint16_t extension_time_ticks=0; // extension time Timer1 ticks (0.5us)
volatile uint16_t perc_inc=0; // percentuale di incremento (letta dalla mappa a seconda di RPM, THR, TIM)
uint8_t incrementi_rpm[INJ_INCR_RPM_MAPS_SIZE]; // increments, based on rpm, used by SW
uint8_t incrementi_thr[INJ_INCR_THR_MAPS_SIZE]; // increments depending on throttle
const uint16_t incrementi_rpm_brkpts[] = {2600, 3112, 4136, 5160, 7208, 11304, 15400, 23592}; // breakpoints, size should be INJ_INCR_RPM_MAPS_SIZE
const uint8_t incrementi_rpm_shifts[] = {9, 10, 10, 11, 12, 12, 13}; // breakpoints, size should be (INJ_INCR_RPM_MAPS_SIZE-1)
volatile bool INJ_safety_inj_turned_off = false; // Injector was not turned OFF since the last function call
volatile uint8_t engine_running_flag = 0; // Becomes ON when the engine is cranked
volatile uint8_t INJ_exec_time_1 = 0; // tempo di esecuzione Inj ON
volatile uint8_t INJ_exec_time_2 = 0; // tempo di esecuzione Inj OFF

// Buffer variables, to store data before sending on Serial or storing on SD
volatile uint16_t injection_counter_buffer=0; // counts the combustion cycles
volatile uint16_t delta_time_tick_buffer=0; // distanza tra due iniezioni (2rpm) buffer
volatile uint16_t delta_inj_tick_buffer=0; // tempo di iniezione input buffer
volatile uint16_t throttle_buffer=0; // tensione del throttle pin 0-1023 buffer
volatile uint16_t lambda_buffer=0; // lambda sensor voltage
volatile uint16_t extension_time_ticks_buffer=0; // extension time ticks
volatile uint8_t buffer_busy=0; // viene attivato quando sta mandando fuori i dati in seriale

enum INJ_signal_trigger_mode_enum{
	WAIT_FOR_ON = 0,
	WAIT_FOR_OFF
};
volatile INJ_signal_trigger_mode_enum INJ_signal_trigger_mode = WAIT_FOR_ON; // waiting for the injector to turn ON


// FUNCTIONS

void INJmgr_class::begin(){
	
	// INJ pin input output settings
	pinMode(IN_INJ_PIN, INPUT_PULLUP);
	pinMode(OUT_INJ_PIN, OUTPUT);
	digitalWrite(OUT_INJ_PIN, LOW);
	
	// INJ pin input interrupt settings (INT0 and Timer1)
	PCICR |=  1 << PCIE2; // PCIE2 (Port D)
	#if FUELINO_HW_VERSION == 1
		PCMSK2 |= 1 << PCINT18; // PCINT18 (Port D Pin 2)
	#elif FUELINO_HW_VERSION == 2
		PCMSK2 |= 1 << PCINT22; // PCINT22 (Port D Pin 6)
	#endif
	PCIFR |= 1 << PCIF2; // PCIF2 (Port D)
	Timer1.initialize(); // Initializes Timer1 for injection time management
	
}


// Returns the number of tick counts of Timer0 (1 tick = 4 us, since f=16MHz and prescaler = 64). 1 Overflow happens each 256 * 4 us = 1024 us
// The returned data size is 2 bytes (16 bit), allowing to count up to 262144 us
extern volatile unsigned long timer0_overflow_count; // needed to read the microseconds counter
uint16_t INJmgr_class::Timer0_tick_counts() {
	
	uint8_t m;
	uint8_t t;
	
	uint8_t oldSREG = SREG;
	cli();
	m = (uint8_t)(timer0_overflow_count & 0xFF);
	t = TCNT0;
	if ((TIFR0 & _BV(TOV0)) && (t < 255)) m++;
	SREG = oldSREG;
	
	return ((m << 8) + t);
	
}


// Interpolates the thr map and caculates, as output, the injection time increment (%). 2 bytes. Resolution: 2e16 = 100%. Therefore, MSB (2e15) = 50%. Therefore 0xFFFF corresponds to a bit less than 100%.
uint16_t INJmgr_class::interpolate_thr_map(){
	
	// Finds the starting index
	uint16_t throttle_tmp = ADCmgr_throttle_signal_read(); // Reads throttle signal (10 bits = 0 .. 1023)
	uint8_t index_low = (uint8_t)(throttle_tmp >> 7) & 0x07; // divide by 128. 0x07 -> to make sure that the index is not over 7 (INJ_INCR_THR_MAPS_SIZE - 1)
	
	// Finds the basic increment, and DeltaY increment (%)
	uint8_t y0 = incrementi_thr[index_low]; // 1e8 = 50%
	uint16_t temp_result = y0 << 7; // 1 byte data: 2e8 = 50%. 2 byte data: 2e15 = 50%. Therefore, it is needed to shift by 7 positions.
	if (index_low >= (INJ_INCR_THR_MAPS_SIZE - 1)){ // Max value of the array
		return temp_result; // Saturated value
	}
	uint8_t y1 = incrementi_thr[index_low + 1]; // Increment of next element
	uint8_t dy; // "dy" for slope calculation
	bool positive; // "dy" is positive flag
	if (y1 >= y0){
		dy = y1 - y0; // y increment positive
		positive=true;
	}else{
		dy = y0 - y1; // y increment negative
		positive=false;
	}
	
	// Finds the DeltaX remaining increment, multiplies it by the DeltaY
	uint16_t rem_x = (throttle_tmp - ((uint16_t)index_low << 7)); // This should be a number between 0 and 127
	uint16_t mult_tmp = (uint16_t)((uint32_t)rem_x * (uint32_t)dy); // No need to shift after, because it would be 7 right (division) and 7 left (multiplication)
	
	// Performs addition, depending on the sign
	if (positive == true){
		return (temp_result + mult_tmp); // ==> 1e16 = 100%
	}else{
		return (temp_result - mult_tmp); // ==> 1e16 = 100%
	}

}


// Interpolates the rpm map and calculates, as output, the injection time increment (%). 2 bytes. Resolution: 2e16 = 100%. Therefore, MSB (2e15) = 50%. Therefore 0xFFFF corresponds to a bit less than 100%.
uint16_t INJmgr_class::interpolate_rpm_map(){
	
	// Search for proper index
	if (delta_time_tick <= incrementi_rpm_brkpts[0]) return ((uint16_t)incrementi_rpm[0] << 7); // return array lowest value (the x value was lower than all the elements)
	uint8_t index; // index found	
	for (index=1; index<INJ_INCR_RPM_MAPS_SIZE; index++){ // scan all breakpoints
		if (delta_time_tick < incrementi_rpm_brkpts[index]){ // x lower is higher than next element
			index--; // previous index
			break; // exit cycle
		}
	}
	if (index >= (INJ_INCR_RPM_MAPS_SIZE - 1)) return ((uint16_t)incrementi_rpm[INJ_INCR_RPM_MAPS_SIZE-1] << 7); // return array highest value (the x value was higher than all the elements)
	
	// Performs subtraction and multiplication of the remaining part
	uint16_t rem_x = delta_time_tick - incrementi_rpm_brkpts[index]; // remaining part of x value
	uint8_t y0 = incrementi_rpm[index]; // Low Y element
	uint8_t y1 = incrementi_rpm[index+1]; // High Y element
	uint8_t dy; // "dy" for slope calculation
	bool positive; // "dy" is positive flag
	if (y1 >= y0){
		dy = y1 - y0; // y increment positive
		positive=true;
	}else{
		dy = y0 - y1; // y increment negative
		positive=false;
	}
	uint16_t mult_tmp = (uint16_t)((uint32_t)rem_x * (uint32_t)dy);
	
	// Performs division and resolution shifting
	uint8_t div_shifts = incrementi_rpm_shifts[index]; // number of shifts on the right (division by "dx")
	if (div_shifts >= 7){ // Division is bigger than multiplication
		for (uint8_t i=0; i<(div_shifts - 7); i++){ // shift right "div_shift" times. "7" multiplication is to present the value in "10e16 = 100%" format
			mult_tmp = mult_tmp >> 1; 
		}
	}else{
		for (uint8_t i=0; i<(7 - div_shifts); i++){ // shift right "div_shift" times. "7" multiplication is to present the value in "10e16 = 100%" format
			mult_tmp = mult_tmp << 1; 
		}
	}
	
	// Performs addition
	if (positive == true){
		return ((uint16_t)y0 << 7) + mult_tmp; // 1e8 = 50% ==> 1e16 = 100%
	}else{
		return ((uint16_t)y0 << 7) - mult_tmp; // 1e8 = 50% ==> 1e16 = 100%
	}

}


// Updates the info buffer for logger / serial communication
void INJmgr_class::update_info_for_logger() {
	if (!buffer_busy) { // the ECU is not using the buffer to send data to the logger
		injection_counter_buffer = injection_counter; // injection counter is increased
		delta_time_tick_buffer = delta_time_tick; // distanza tra due iniezioni (2rpm) buffer
		delta_inj_tick_buffer = delta_inj_tick; // tempo di iniezione input buffer
		extension_time_ticks_buffer = extension_time_ticks; // Injection time externsion time (Timer)
	}
}


// Deactivates the injector when Timer1 interrupt happens
void deactivate_inj(){
	Timer1.stop(); // stops the timer
	digitalWrite(OUT_INJ_PIN, LOW); // shuts down injector
	INJ_safety_inj_turned_off = true; // Injector turned OFF (this flag is used for Safety check)
	Timer1.detachInterrupt(); // Removes Timer 1 interrupt
	INJmgr.update_info_for_logger(); // Updates info buffer for logger / serial
}


// Acquires signal from throttle sensor
void INJmgr_class::analog_digital_signals_acquisition(){
	if (!buffer_busy){ // if the buffer is not being used for reading, update it
		throttle_buffer = ADCmgr_throttle_signal_read(); // update buffer
		lambda_buffer = ADCmgr_lambda_signal_read(); // update buffer
	}
}


// This function has the purpose of checking if the injector has been properly deactivated
void INJmgr_class::safety_check(uint32_t time_now_ms){
	
	// Time check (checks if enough time has expired since last time)
	uint16_t time_now_ticks = (uint16_t)time_now_ms; // need only the lower 2 bytes: 0-65535 ms
	if ((time_now_ticks - INJ_safety_check_last_exec_time) >= INJ_SAFETY_EXEC_TIME){
		INJ_safety_check_last_exec_time = time_now_ticks; // save current time
	
		// Checks if, from last function call, the interrupt properly turned on the flag
		if (INJ_safety_inj_turned_off == false){ // If this flag is active, interrupt did not set the injector to OFF
			INJ_safety_err_counter++; // Increase error counter
		}else{
			INJ_safety_err_counter = 0; // No error, reset the counter
		}
		if(INJ_safety_err_counter >= INJ_SAFETY_MAX_ERR){ // If maximum error number is reached
			digitalWrite(OUT_INJ_PIN, LOW); // shuts down injector
			INJ_safety_err_counter = 0;
			INJ_signal_trigger_mode = WAIT_FOR_ON;
		}
		INJ_safety_inj_turned_off = false; // re-triggers the flag
	
	}
}


// Evaluates if the engine working point is in steady state conditions
void INJmgr_class::steady_state_eval(uint32_t time_now_ms){
	
	// Time check
	uint16_t time_now_tmp = (uint16_t)time_now_ms; // need only the lower 2 bytes: 0-65535 ms
	if ((time_now_tmp - steady_state_timer_last_exec) >= INJ_STEADY_STATE_MIN_TIME_BTW_TASKS){
		steady_state_timer_last_exec = time_now_tmp; // update current execution time
		
		// Buffering engine variables
		buffer_busy = 1; // Locks the buffer access (semaphore)
		uint16_t combustion_num_val = injection_counter_buffer; // combustion-injections counter
		uint16_t delta_ticks_val = delta_time_tick_buffer; // 2 engine rotations ticks (1 tick = 4us)
		uint16_t injec_ticks_val = delta_inj_tick_buffer; // injection ticks
		uint16_t throttle_val = throttle_buffer; // throttle sensor signal
		buffer_busy = 0; // Unlocks the buffer again
		
		// Determine if engine steady conditions are verified or not
		uint8_t steady_state_tmp = 0; // Initialized to "not steady state"
		if (combustion_num_val != steady_state_combustion_number_old){ // combustion cycle number changed
			if ((delta_ticks_val >= INJ_STEADY_STATE_DELTA_TICKS_MIN) && (delta_ticks_val <= INJ_STEADY_STATE_DELTA_TICKS_MAX)){ // rpm correct range
				if ((throttle_val >= INJ_STEADY_STATE_THROTTLE_MIN) && (throttle_val <= INJ_STEADY_STATE_THROTTLE_MAX)){ // throttle correct range
					uint16_t rpm_delta_max = steady_state_delta_ticks_old >> 7; // 2e7 = 128, previous value divided by 128 (0.75% variation)
					if ((delta_ticks_val >= (steady_state_delta_ticks_old - rpm_delta_max)) && (delta_ticks_val <= (steady_state_delta_ticks_old + rpm_delta_max))){ // rpm variation range check
						if ((throttle_val >= (steady_state_throttle_old - INJ_STEADY_STATE_THROTTLE_DELTA_MAX)) && (throttle_val <= (steady_state_throttle_old + INJ_STEADY_STATE_THROTTLE_DELTA_MAX))){ // throttle variation range check
							steady_state_tmp = 1; // Steady state condition
						}
						engine_running_flag = 1; // Engine can be considered running
					}
				}
			}
		}
		
		// Request the proper prescaler to ADCmgr module ("0" means acquisition OFF/finished)
		if (steady_state_tmp){ // Setting the prescaler to the proper calculated value, only if the ADCmgr and SDmgr are not working
			uint8_t i=1; // Next: write bytes 1-8
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)(combustion_num_val & 0xff); // LSB
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)((combustion_num_val >> 8) & 0xff); // MSB
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)(delta_ticks_val & 0xff); // LSB
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)((delta_ticks_val >> 8) & 0xff); // MSB
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)(injec_ticks_val & 0xff); // LSB
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)((injec_ticks_val >> 8) & 0xff); // MSB
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)(throttle_val & 0xff); // LSB
			ADCmgr_lambda_acq_buf[i++] = (uint8_t)((throttle_val >> 8) & 0xff); // MSB
			if (delta_ticks_val > 8000) steady_state_tmp = 2;
			if (delta_ticks_val > 16000) steady_state_tmp = 3;
			if (delta_ticks_val > 24000) steady_state_tmp = 4;
		}
		steady_state_prescaler_tgt = steady_state_tmp;
		
		// Save old values
		steady_state_combustion_number_old = combustion_num_val; // injection-combustion number old value
		steady_state_throttle_old = throttle_val; // throttle old value
		steady_state_delta_ticks_old = delta_ticks_val; // 2 engine rotations old value
	
	}
}


// Interrupt pin status changes (Original ECU injector command pin)
ISR(PCINT2_vect){
	
	// For execution time measurement
	uint16_t INJ_exec_time_start = INJmgr.Timer0_tick_counts(); // 1 tick = 4us
	
	// Evaluate the status of the INJ input pin
	uint8_t INJ_signal_status; // Injector pin status (Low = ON, High = OFF)
	if (engine_running_flag){ // Cranking finished
		INJ_signal_status = (PIND & _BV(IN_INJ_PIN)); // acquire status of digital input (LOW = ON, HIGH = OFF) on injector input pin
	}else{ // During cranking, signal is noisy. The following algorithm filters the signal. Only in case the signal is reliable, it is used
		uint8_t pin_status_cnt = 0;
		for (uint8_t i=0; i<INJ_CRANKING_INJ_PIN_FILT_MEASURES; i++){ // Performs many measurements and accumulates the status (0 or 1) in a counter
			pin_status_cnt += ((PIND & _BV(IN_INJ_PIN)) >> IN_INJ_PIN);
		}
		if (pin_status_cnt <= INJ_CRANKING_INJ_PIN_FILT_LOW){
			INJ_signal_status = 0; // Low (Injector ON)
		}
		else if (pin_status_cnt >= INJ_CRANKING_INJ_PIN_FILT_HIGH){
			INJ_signal_status = 1; // High (Injector OFF)
		}
		else{
			return; // pulses are just noise
		}
	}

	// Original ECU is enabling injector (Injector valve will open and gasoline flows)
	if (INJ_signal_status == 0){
		if (INJ_signal_trigger_mode == WAIT_FOR_ON){ // I was waiting for this OFF -> ON transition
			digitalWrite(OUT_INJ_PIN, HIGH); // activates injector
			delta_time_tick = INJ_exec_time_start - inj_start_tick_last; // calculates Timer0 ticks (4us) between 2 consecutive injections (2 rpm)
			perc_inc = INJmgr.interpolate_rpm_map(); // returns the interpolated increment based on rpm
			inj_start_tick_last = INJ_exec_time_start; // saves old time stamp
			injection_counter++; // increases the combustion cycles
			INJ_signal_trigger_mode = WAIT_FOR_OFF; // Next cycle, wait for ON -> OFF transition
		}
	}

	// Original ECU is disabling injector (Injector valve will close and gasoline stops, after Timer1 expires)
	else{ // OFF
		if (INJ_signal_trigger_mode == WAIT_FOR_OFF){ // I was waiting for this ON -> OFF transition
			delta_inj_tick = INJ_exec_time_start - inj_start_tick_last; // delta injetion time, in Timer0 tiks (4 us)
			bool extension_timer_activated = false; // Timer1 not yet initialized
			if ((delta_inj_tick >= INJ_TIME_TICKS_MIN) && (delta_inj_tick <= INJ_TIME_TICKS_MAX)) { // injection range check
				// Remove the offset (delay and opening time from measured ticks)
				if (delta_inj_tick > INJ_OPENING_TIME_TICKS) { // remove the opening time from the time measured
					delta_inj_tick -= INJ_OPENING_TIME_TICKS; // real injection time (removed the opening time)
				}
				else {
					delta_inj_tick = 0; // if time is lower than estimated injector opening time, real injection time is considered 0
				}
				// Calculate final percentage increment
				perc_inc+=INJmgr.interpolate_thr_map(); // percentage increment depending on throttle position (1e16 = 100%)
				if (perc_inc > MAX_PERCENTAGE_INJ) perc_inc = MAX_PERCENTAGE_INJ; // saturates the maximum injection percentage, for safety
				extension_time_ticks = (uint16_t)(((uint32_t)delta_inj_tick * (uint32_t)perc_inc) >> (16 - 3)); // calculates extension time, in Timer1 ticks (0.5us). Need to shift 16 pos (because 1e16 = 100%)
				if ((extension_time_ticks >= MIN_EXTENSION_TIME_TICKS) && (extension_time_ticks <= MAX_EXTENSION_TIME_TICKS)){ // Checks if the extension time ticks (Timer1) is acceptable
					if (extension_time_ticks > INJ_CALCULATION_TIME_TICKS) { // Removes the calculation time requested
						extension_time_ticks -= INJ_CALCULATION_TIME_TICKS; // real extension time
						Timer1.setPeriod(extension_time_ticks); // Sets the timer (Timer1) overflow threshold
						Timer1.attachInterrupt(deactivate_inj); // Attachs Timer1 interrupt
						Timer1.start(); // Starts the timer
						extension_timer_activated = true; // Timer has been activated
						INJ_signal_trigger_mode = WAIT_FOR_ON; // Waiting for next ON event
					}
				}
			}
			// In case the previous calculaton stopped somewhere before activating the timer, do the following (deactivate injector, and so on)
			if (extension_timer_activated == false){ // timer was not activated, so need to disable the output now
				digitalWrite(OUT_INJ_PIN, LOW); // shuts down injector
				INJ_safety_inj_turned_off = true; // Injector turned OFF (this flag is used for Safety check)
				extension_time_ticks = 0; // no extension time programmed
				INJ_signal_trigger_mode = WAIT_FOR_ON; // Waiting for next ON event
				INJmgr.update_info_for_logger(); // Updates the data inside buffer
			}
			if ((ADCmgr_lambda_acq_prescaler_max == 0) && (!ADCmgr_lambda_acq_buf_filled)){ // Makes sure that the acquisition is not in progress, and that the buffer is not being written on SD
				ADCmgr_lambda_acq_prescaler_max = INJmgr.steady_state_prescaler_tgt; // Start the Lambda ADC acquisition, if in Steady State condition
				INJmgr.steady_state_prescaler_tgt = 0; // Resets the request flag
			}
		}
	}
	
	// For execution time measurement
	uint16_t INJ_exec_time_end = INJmgr.Timer0_tick_counts(); // 1 tick = 4us
	uint16_t INJ_exec_time = INJ_exec_time_end - INJ_exec_time_start; // Measure interrupt execution time
	if (!INJ_signal_status){ // Low = Injector ON
		INJ_exec_time_1 = (uint8_t)INJ_exec_time; // ON
	}else{
		INJ_exec_time_2 = (uint8_t)INJ_exec_time; // OFF
	}
	
	//PCIFR |= 1 << PCIF2; // PCIF2 (Port D) | Clears any interrupt request on Port D, as double check, to filter any noise
	
}

INJmgr_class INJmgr;

#endif