#ifndef CompileOptions_h
#define CompileOptions_h

// Arduino internal options
#define FUELINO_HW_VERSION 2 // HW version of Fuelino. Fuelino V1 does not have SWseriale, and also, for injector management, input and output pins are different. For Fuelino Proto3, please enter "2"
#define ENABLE_BUILT_IN_HW_SERIAL 1 // Enables the communication HW Serial port (USB connection with PC) on pins 0 and 1. Default is "1". I suggest to keep it as "1", since it does not create problems.

// External modules
#define SD_MODULE_PRESENT 1 // SD Card module present
#define DISPLAY_PRESENT 0 // Display module on I2C
#define MPU6050_PRESENT 1 // IMU module on I2C
#define GPS_PRESENT 1 // GPS module on SW Serial
#define BLUETOOTH_PRESENT 0 // Enables packets forwarding (sending and receiving) through SW Serial, in case FUELINO_HW_VERSION>=2, and a Bluetoooth (or Wifi module) is connected on SWseriale

// Main Loop execution time
#define LOOP_MIN_EXEC_TIME 25 // Main Loop minimum execution time [ms]

#endif