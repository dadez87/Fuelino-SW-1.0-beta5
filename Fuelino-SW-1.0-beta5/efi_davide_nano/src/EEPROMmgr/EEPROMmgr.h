#ifndef EEPROMmgr_h
#define EEPROMmgr_h

#include <EEPROM.h>
#include <Arduino.h>

// Functions of EEPROMmgr to be exported
extern void EEPROM_initialize();
extern uint8_t EEPROM_write_standard_values(uint8_t data_number);
extern uint16_t EEPROM_SD_file_num_rw();
extern void EEPROM_write_RAM_map_to_EEPROM(uint8_t map_number_req);

extern uint8_t bat_check_inhibit();
extern uint8_t eng_log_inhibit();
extern uint8_t gps_log_inhibit();
extern uint8_t imu_log_inhibit();
extern uint8_t lam_log_inhibit();

#endif