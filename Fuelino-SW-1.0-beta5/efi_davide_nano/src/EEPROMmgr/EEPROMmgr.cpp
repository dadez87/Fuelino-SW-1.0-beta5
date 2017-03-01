#ifndef EEPROMmgr_cpp
#define EEPROMmgr_cpp

#include "EEPROMmgr.h"
#include "../INJmgr/INJmgr.h" // To read and write injection percentage increase calibration maps

// EEPROM memory addresses table:
#define INJ_INCR_RPM_MAPS_START 0 // start address (0) -> 8x2 bytes used for this map = 16
#define INJ_INCR_THR_MAPS_START 2*INJ_INCR_RPM_MAPS_SIZE // start address (16) -> 16x2 bytes used for this map = 32
#define SD_FILE_NUM_ADDR 64 // address of file number (4 bytes used)
#define EEPROM_CONFIG_WORD_ADDR 68 // address of Configuration Word
#define BATTERY_CHECK_BYPASS_BIT 0
#define ENG_DATA_LOG_BYPASS_BIT 1
#define GPS_DATA_LOG_BYPASS_BIT 2
#define IMU_DATA_LOG_BYPASS_BIT 3
#define LAM_DATA_LOG_BYPASS_BIT 4

uint8_t EEPROM_config_word = 0; // Configuration word saved into EEPROM ("0x00" is the temporary value, in case of fault)

uint8_t bat_check_inhibit(){ return ((EEPROM_config_word & (1 << BATTERY_CHECK_BYPASS_BIT)) >> BATTERY_CHECK_BYPASS_BIT); }
uint8_t eng_log_inhibit(){ return ((EEPROM_config_word & (1 << ENG_DATA_LOG_BYPASS_BIT)) >> ENG_DATA_LOG_BYPASS_BIT); }
uint8_t gps_log_inhibit(){ return ((EEPROM_config_word & (1 << GPS_DATA_LOG_BYPASS_BIT)) >> GPS_DATA_LOG_BYPASS_BIT); }
uint8_t imu_log_inhibit(){ return ((EEPROM_config_word & (1 << IMU_DATA_LOG_BYPASS_BIT)) >> IMU_DATA_LOG_BYPASS_BIT); }
uint8_t lam_log_inhibit(){ return ((EEPROM_config_word & (1 << LAM_DATA_LOG_BYPASS_BIT)) >> LAM_DATA_LOG_BYPASS_BIT); }

// Loads the configuration word from the EEPROM
void EEPROM_load_config_word(){
	uint8_t config_word_tmp = EEPROM.read(EEPROM_CONFIG_WORD_ADDR); // original value
	uint8_t config_word_tmp_red = EEPROM.read(EEPROM_CONFIG_WORD_ADDR+1); // redundant value
	if (config_word_tmp_red == ((uint8_t)255 - config_word_tmp)){ // checksum check OK
		EEPROM_config_word = config_word_tmp;
	}
}


// Writes injection standard values into EEPROM
uint8_t EEPROM_write_standard_values(uint8_t data_number){
  uint8_t i;
  if ((data_number ==0) || (data_number ==INJ_MAPS_TOTAL_NUM)){ // RPM map
	  for (i=0; i<INJ_INCR_RPM_MAPS_SIZE; i++){
		  EEPROM.write(INJ_INCR_RPM_MAPS_START+i, INJ_INCREMENT_RPM_STD); //main data
		  EEPROM.write(INJ_INCR_RPM_MAPS_START+INJ_INCR_RPM_MAPS_SIZE+i, (uint8_t)255 - INJ_INCREMENT_RPM_STD); //for redundancy, this is a copy of the main data
		  incrementi_rpm[i]=INJ_INCREMENT_RPM_STD; // prende il valore standard
	  }
  }
  if ((data_number ==1) || (data_number ==INJ_MAPS_TOTAL_NUM)){ // THR map
	for (i=0; i<INJ_INCR_THR_MAPS_SIZE; i++){
		EEPROM.write(INJ_INCR_THR_MAPS_START+i, INJ_INCREMENT_THR_STD); //main data
		EEPROM.write(INJ_INCR_THR_MAPS_START+INJ_INCR_THR_MAPS_SIZE+i, (uint8_t)255 - INJ_INCREMENT_THR_STD); //for redundancy, this is a copy of the main data
		incrementi_thr[i]=INJ_INCREMENT_THR_STD; // prende il valore standard
	}
  }
  return 0;
}


// Loads calibration values for injection. If an error is found, standard values are written
uint8_t EEPROM_load_calib(){
	uint8_t error_flag = 0;
	uint8_t i;
	// Imports "incrementi_rpm"
	for (i=0; i<INJ_INCR_RPM_MAPS_SIZE; i++){
		if (EEPROM.read(INJ_INCR_RPM_MAPS_START+INJ_INCR_RPM_MAPS_SIZE+i) != ((uint8_t)255 - (uint8_t)EEPROM.read(INJ_INCR_RPM_MAPS_START+i))){ // redundancy check NG
			error_flag=1;
		}
		else{ // OK
			incrementi_rpm[i]=EEPROM.read(INJ_INCR_RPM_MAPS_START+i);
		}
	}
	// Imports "incrementi_thr"
	for (i=0; i<INJ_INCR_THR_MAPS_SIZE; i++){
		if (EEPROM.read(INJ_INCR_THR_MAPS_START+INJ_INCR_THR_MAPS_SIZE+i) != ((uint8_t)255 - (uint8_t)EEPROM.read(INJ_INCR_THR_MAPS_START+i))){ // redundancy check NG
			error_flag=1;
		}
		else{ // OK
			incrementi_thr[i]=EEPROM.read(INJ_INCR_THR_MAPS_START+i);
		}
	}
	if (error_flag){
		EEPROM_write_standard_values(INJ_MAPS_TOTAL_NUM); // Writes the standard values
		return 0; // Not OK
	}
	return 1; // OK
}


// Reads data from EEPROM, and initializes them in RAM memory
void EEPROM_initialize(){
	EEPROM_load_config_word();
	EEPROM_load_calib();
}


// Writes a map from RAM into EEPROM memory
void EEPROM_write_RAM_map_to_EEPROM(uint8_t map_number_req){
	if ((map_number_req ==0) || (map_number_req ==INJ_MAPS_TOTAL_NUM)){ // rpm
		for (uint8_t i=0; i<INJ_INCR_RPM_MAPS_SIZE;i++){
			EEPROM.write(INJ_INCR_RPM_MAPS_START+i, incrementi_rpm[i]); //main data
			EEPROM.write(INJ_INCR_RPM_MAPS_START+INJ_INCR_RPM_MAPS_SIZE+i, (uint8_t)255 - incrementi_rpm[i]); //for redundancy, this is a copy of the main data
		}
	}
	if ((map_number_req ==1) || (map_number_req ==INJ_MAPS_TOTAL_NUM)){ // throttle
		for (uint8_t i=0; i<INJ_INCR_THR_MAPS_SIZE;i++){
			EEPROM.write(INJ_INCR_THR_MAPS_START+i, incrementi_thr[i]); //main data
			EEPROM.write(INJ_INCR_THR_MAPS_START+INJ_INCR_THR_MAPS_SIZE+i, (uint8_t)255 - incrementi_thr[i]); //for redundancy, this is a copy of the main data
		}
	}
}


// Reads file number into EEPROM, increases it (+1), writes the increased number into EEPROM, and returns that value. In case of error, "0" is used.
uint16_t EEPROM_SD_file_num_rw(){
	
	// EEPROM values read
	uint8_t valoreL = EEPROM.read(SD_FILE_NUM_ADDR); // original value
	uint8_t valoreH = EEPROM.read(SD_FILE_NUM_ADDR+1); // original value
	uint8_t valore_redL = EEPROM.read(SD_FILE_NUM_ADDR+2); // redundancy
	uint8_t valore_redH = EEPROM.read(SD_FILE_NUM_ADDR+3); // redundancy
	
	// Deciding values
	if ((valore_redH == ((uint8_t)255 - valoreH)) && (valore_redL == ((uint8_t)255 - valoreL))){ // checksum check OK
		uint16_t valore = ((uint16_t)valoreL + ((uint16_t)valoreH << 8));
		valore++; // increase value
		valoreL = (uint8_t)(valore & (uint16_t)0x00FF); // low
		valoreH = (uint8_t)(valore >> 8); // high
	}else{ // initialization (standard values)
		valoreL = (uint8_t)0;
		valoreH = (uint8_t)0;
	}
	valore_redL=(uint8_t)255 - valoreL;
	valore_redH=(uint8_t)255 - valoreH;
	
	// Writing new values into EEPROM
	EEPROM.write(SD_FILE_NUM_ADDR, valoreL);
	EEPROM.write(SD_FILE_NUM_ADDR+1, valoreH);
	EEPROM.write(SD_FILE_NUM_ADDR+2, valore_redL);
	EEPROM.write(SD_FILE_NUM_ADDR+3, valore_redH);
	
	return ((uint16_t)valoreL + ((uint16_t)valoreH << 8));
}

#endif