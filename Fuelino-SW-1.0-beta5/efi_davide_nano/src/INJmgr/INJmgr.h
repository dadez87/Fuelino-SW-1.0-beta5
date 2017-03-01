#ifndef INJmgr_h
#define INJmgr_h

#include <Arduino.h>
#include "../compile_options.h"

#if FUELINO_HW_VERSION == 1
	#define IN_INJ_PIN 2 // input pin injector from ECU
	#define OUT_INJ_PIN 9 // output pin injector to physical injector
#elif FUELINO_HW_VERSION == 2
	#define IN_INJ_PIN 6 // input pin injector from ECU
	#define OUT_INJ_PIN 8 // output pin injector to physical injector
#endif

#define INJ_TIME_TICKS_MIN (uint16_t)50 // minima iniezione 200us (1 Timer0 tick = 4us)
#define INJ_TIME_TICKS_MAX (uint16_t)2000 // massima iniezione 8ms = 8000us (1 Timer0 tick = 4us)
#define MAX_PERCENTAGE_INJ (uint16_t)32768 // maximum increment percentage (for safety) [32768 corresponds to 50% increment, because 2e16 = 100%]
#define MIN_EXTENSION_TIME_TICKS (uint16_t)100 // minimum time of Timer1 extension time ticks (0.5us), for injection time extension [100 = 50us]
#define MAX_EXTENSION_TIME_TICKS (uint16_t)6000 // minimum time of Timer1 extension time ticks (0.5us), for injection time extension [6000 = 3000us]
#define INJ_OPENING_TIME_TICKS (uint16_t)0 // time ticks (4us) required for the injector to open [example: 6 == 24us]
#define INJ_CALCULATION_TIME_TICKS (uint16_t)54 // time ticks (0.5us) required for calculation of extension time [example: 60 == 30us]
#define INJ_INCREMENT_RPM_STD (uint8_t)128 // Injection increment standard (50/256 %)
#define INJ_INCREMENT_THR_STD (uint8_t)0 // Injection increment standard (50/256 %)
#define INJ_INCR_RPM_MAPS_SIZE 8 // Engine Speed compensation map size
#define INJ_INCR_THR_MAPS_SIZE 8 // Throttle compensation map size [if you change this, you should also change "index_thr_tmp" calculation]
#define INJ_MAPS_TOTAL_NUM 2 // total number of calibration maps is 2 ("incrementi_rpm" and "incrementi_thr")
#define INJ_SAFETY_MAX_ERR (uint8_t)10 // maximum number of tolerable Safety errors, then turn OFF the injector
#define INJ_SAFETY_EXEC_TIME (uint16_t)100 // safety execution time (ms)
#define INJ_STEADY_STATE_MIN_TIME_BTW_TASKS (uint16_t)500 // minimum time between tasks (ms)
#define INJ_STEADY_STATE_DELTA_TICKS_MIN (uint16_t)2500 // 1 tick = 4us. 2500 = 10000us = 12000rpm
#define INJ_STEADY_STATE_DELTA_TICKS_MAX (uint16_t)40000 // 1 tick = 4us. 40000 = 160000us = 750rpm
#define INJ_STEADY_STATE_THROTTLE_MIN (uint16_t)0 // Minimum throttle for steady state evaluation
#define INJ_STEADY_STATE_THROTTLE_MAX (uint16_t)1023 // Maximum throttle for steady state evaluation
#define INJ_STEADY_STATE_THROTTLE_DELTA_MAX (uint16_t)16 // Maximum throttle variation allowed (1/64 = 1.5% variation)
#define INJ_CRANKING_INJ_PIN_FILT_MEASURES (uint8_t)64 // Total number of sampling, for filtering, during cranking phase
#define INJ_CRANKING_INJ_PIN_FILT_LOW (uint8_t)16 // Threshold value to be considered "low" (Injector ON)
#define INJ_CRANKING_INJ_PIN_FILT_HIGH (uint8_t)48 // Threshold value to be considered "high" (Injector OFF)

// Variables to be exported
extern uint8_t incrementi_rpm[]; // increments depending on rpm
extern uint8_t incrementi_thr[]; // increments depending on throttle

// Variables related to the Engine Data buffer (for SD card logging)
extern volatile uint16_t injection_counter_buffer; // counts the combustion cycles
extern volatile uint16_t delta_time_tick_buffer; // time between 2 consecutive injections (2rpm) buffer [1 tick = 4us]
extern volatile uint16_t delta_inj_tick_buffer; // injection time buffer [1 tick = 4us]
extern volatile uint16_t throttle_buffer; // voltage on throttle pin 0-1023 buffer
extern volatile uint16_t lambda_buffer; // lambda sensor voltage
extern volatile uint16_t extension_time_ticks_buffer; // extension time Timer1 ticks (0.5us)
extern volatile uint8_t INJ_exec_time_1; // execution time for the interrupt (ON)
extern volatile uint8_t INJ_exec_time_2; // execution time for the interrupt (OFF)
extern volatile uint8_t buffer_busy; // activated when SD packet (and serial packet) are built, to avoid corruption

class INJmgr_class{
	
	public:
		void begin(); // Called in MAIN Setup
		uint16_t Timer0_tick_counts();
		uint16_t interpolate_rpm_map();
		uint16_t interpolate_thr_map();
		void update_info_for_logger();
		void analog_digital_signals_acquisition();
		void safety_check(uint32_t time_now_ms);
		void steady_state_eval(uint32_t time_now_ms);
		
		uint8_t INJ_safety_err_counter = 0; // Safety error counter
		uint16_t INJ_safety_check_last_exec_time = 0; // Last time that safety check was executed (1 LSB = 4us)
		
		// Variables for engine steady state condition determination
		uint16_t steady_state_throttle_old = 0; // Old value of throttle (0..1023), for steady state evaluation
		uint16_t steady_state_delta_ticks_old = 0; // Old value of delta ticks (2 engine rotations, 1 tick 4us), for steady state evaluation
		uint16_t steady_state_combustion_number_old = 0xFFFF; // Old value of combustion cycle (must be set to 0xFFFF since this value is different than 0)
		uint8_t steady_state_prescaler_tgt = 0; // prescaler target
		uint16_t steady_state_timer_last_exec = 0; // counter for steady state execution
		
};

extern INJmgr_class INJmgr;

#endif