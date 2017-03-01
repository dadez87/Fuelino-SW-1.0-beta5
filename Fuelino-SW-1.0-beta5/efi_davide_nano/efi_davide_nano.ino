// Created by Davide Cavaliere
// E-mail: dadez87@gmail.com
// Website: www.monocilindro.com
// 28 February 2017
// Fuelino SW1.0 beta5
// Compiles successfully with Arduino IDE 1.8.0

#include "src/compile_options.h" // Compile options
#include "src/EEPROMmgr/EEPROMmgr.h" // EEPROM memory management
#include "src/INJmgr/INJmgr.h" // Injection and physical input/outputs management (except for A0 - A7)
#include "src/COMMmgr/COMMmgr.h" // Serial Ports Communication management
#include "src/SDmgr/SDmgr.h" // SD memory management
#include "src/GPSmgr/GPSmgr.h" // GPS manager
#include "src/ADCmgr/ADCmgr.h" // ADC manager (Analog Inputs A0 - A7)
#include "src/MPU6050mgr/MPU6050mgr.h" // IMU module (if present)

bool first_loop=true; // first time to execute the loop
uint16_t time_last_gate = 0; // for loop minimum time check

// Function called during SD card write idling, and in Main Loop while waiting
void fuelino_yield(unsigned long time_now_ms){
  INJmgr.safety_check(time_now_ms); // Safety checks (checks if, from last function call, the injector has been deactivated at least one time)
  MPU6050mgr.manager(time_now_ms); // IMU communication manager
  INJmgr.analog_digital_signals_acquisition(); // Acquires throttle position sensor and lambda signals
  //delay(1);
}

void setup(){
  COMM_begin(); // Initializes serial communication ports
  INJmgr.begin(); // Injection and signal IO management
  EEPROM_initialize(); // EEPROM memory check, loads calibration maps and config word
  ADCmgr_init(); // Initializes the ADC
  MPU6050mgr.begin(); // Initializes IMU module 
}

void loop() {

  // This part is executed only at first cycle loop
  if (first_loop == true){
    #if (GPS_PRESENT == 1) && (FUELINO_HW_VERSION >= 2) && (BLUETOOTH_PRESENT == 0)
    GPS_initialize(); // Initializes GPS module communication (this function uses "delays"), this is why it is put here and not in "Setup"
    #endif
    first_loop = false;
  }

  unsigned long time_now_tmp = millis(); // Time entering the loop 

  // Scheduled functions
  INJmgr.safety_check(time_now_tmp); // Safety checks (checks if, from last function call, the injector has been deactivated at least one time)
  INJmgr.steady_state_eval(time_now_tmp); // Evaluates if engine is working in steady state conditions, to enable Lambda sensor signal logging
  MPU6050mgr.manager(time_now_tmp); // IMU communication manager
  COMM_receive_check(); // Serial Communication manager
  #if (GPS_PRESENT == 1) && (FUELINO_HW_VERSION >= 2) && (BLUETOOTH_PRESENT == 0)
  GPS_manager(); // GPS communication manager
  #endif
  INJmgr.analog_digital_signals_acquisition(); // Acquires throttle position sensor signal
  SDmgr.log_SD_data(); // SD card data logging, as last, after checking all data

  // Loop time check (waiting cycles)
  time_now_tmp = millis(); // read present time again
  uint16_t time_now_tmp_16bit = (uint16_t)(time_now_tmp & 0x0000FFFF); // read present time
  while((time_now_tmp_16bit - time_last_gate) < (uint16_t)LOOP_MIN_EXEC_TIME){ // Dead time from last time still has to expire
    fuelino_yield(time_now_tmp); // Execution takes some ms
    time_now_tmp = millis(); // read present time (32 bits)
    time_now_tmp_16bit = (uint16_t)(time_now_tmp & 0x0000FFFF); // read present time (16 bits)
  }
  time_last_gate = time_now_tmp_16bit; // Save the time when this "while gate" was crossed

}
