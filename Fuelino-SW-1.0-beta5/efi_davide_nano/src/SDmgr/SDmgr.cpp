#ifndef SDmgr_cpp
#define SDmgr_cpp

#include <SDFatYield.h> // SD FAT management (modified SDFat library)
#include "../EEPROMmgr/EEPROMmgr.h" // Need to manage reading from EEPROM (file name, ...)
#include "../INJmgr/INJmgr.h" // INJ manager. To have access to Engine variables (inj time, ...)
#include "../GPSmgr/GPSmgr.h" // GPS manager. To have access to GPS measurements
#include "../MPU6050mgr/MPU6050mgr.h" // IMU manager. To have access to IMU measurements
#include "../ADCmgr/ADCmgr.h" // ADC manager. To have access to Analog readings (Lambda sensor)
#include "SDmgr.h"

#define SD_CS_PIN_NUM A1 // CS pin for SD card (SPI) - Slave Select
#define MAX_WRITE_ERRORS 10 // Maximum consecutive write errors that will cause a re-init. 
#define MAX_DELAY_POWERON 20 // This is used as time delay at power ON before performing an "init"

SdFat SD; // needed to manage SD card

// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {

  // return date using FAT_DATE macro to format fields
  //*date = FAT_DATE(now.year(), now.month(), now.day());
  *date = FAT_DATE(GPS_year, GPS_month, GPS_day);

  // return time using FAT_TIME macro to format fields
  //*time = FAT_TIME(now.hour(), now.minute(), now.second());
  *time = FAT_TIME(GPS_hour, GPS_min, GPS_sec);
}


SDmgr_class::SDmgr_class(){
	
	SD_init_OK = false; // NG
	packet_cnt = 0; // packet counter init value
	write_errors_cnt = 0;
	
}


bool SDmgr_class::begin(){
	
#if SD_MODULE_PRESENT
	
	if (!ADCmgr_battery_status_read() && !bat_check_inhibit()) return false; // return "False" in case battery was OFF, avoiding SD initialization
	
	// Set date time callback function
	SdFile::dateTimeCallback(dateTime);
	
	// SD INITIALIZATION
	if (!SD.begin(SD_CS_PIN_NUM, SPI_HALF_SPEED)) { // CS pin for SD card is pin #10
		SD_init_OK = false; // NG
		return false;
	}
	SD_init_OK = true; // OK

	// Read file name from EEPROM and stores file name as string (8.3 format)
	uint16_t file_number = EEPROM_SD_file_num_rw();
	file_name="fln";
	if (file_number < 10) file_name+='0';
	if (file_number < 100) file_name+='0';
	if (file_number < 1000) file_name+='0';
	if (file_number < 10000) file_name+='0';
	file_name+=(unsigned int)file_number;
	file_name+=".log";
	
	MPU6050mgr.flush_buffer(); // flushes the IMU buffer (sets no data to write)
	
#endif
	
	return true; // Init successful
	
}


bool SDmgr_class::log_SD_data(){

	// Performs SD Initialization, in case it is necessary (in case it was not possible before, or it is the first time to call this function)
	if (SD_init_OK == false) {
		write_errors_cnt++; // this is for delay purposes, to avoid continuous "begin()" calls
		if (write_errors_cnt >= MAX_DELAY_POWERON) {
			begin(); // Initializes SD memory
			write_errors_cnt = 0; // reset counter
		} 
	}

	bool temp_reply = false; // temporary response

	// Preparation of Engine Data array (this is needed also for Serial communication - service protocol)
	uint8_t i=0; // packet bytes counter
	SD_writing_buffer[i++] = 'd';
	SD_writing_buffer[i++] = packet_cnt; // packet counter
	packet_cnt++; // increase packet counter
	unsigned long time_stamp_temp = millis(); // read time stamp
	SD_writing_buffer[i++] = (uint8_t)(time_stamp_temp & 0xff); // LSB
    SD_writing_buffer[i++] = (uint8_t)((time_stamp_temp >> 8) & 0xff);
    SD_writing_buffer[i++] = (uint8_t)((time_stamp_temp >> 16) & 0xff);
    SD_writing_buffer[i++] = (uint8_t)((time_stamp_temp >> 24) & 0xff); // MSB
	buffer_busy = 1; // Locks the buffer access (semaphore)
	SD_writing_buffer[i++] = (uint8_t)(injection_counter_buffer & 0xff); // LSB
	SD_writing_buffer[i++] = (uint8_t)((injection_counter_buffer >> 8) & 0xff); // MSB
    SD_writing_buffer[i++] = (uint8_t)(delta_time_tick_buffer & 0xff); // LSB
	SD_writing_buffer[i++] = (uint8_t)((delta_time_tick_buffer >> 8) & 0xff); // MSB
    SD_writing_buffer[i++] = (uint8_t)(delta_inj_tick_buffer & 0xff); // LSB
    SD_writing_buffer[i++] = (uint8_t)((delta_inj_tick_buffer >> 8) & 0xff); // MSB
    SD_writing_buffer[i++] = (uint8_t)(throttle_buffer & 0xff); // LSB
    SD_writing_buffer[i++] = (uint8_t)((throttle_buffer >> 8) & 0xff); // MSB
	SD_writing_buffer[i++] = (uint8_t)(lambda_buffer & 0xff); // LSB
    SD_writing_buffer[i++] = (uint8_t)((lambda_buffer >> 8) & 0xff); // MSB
	SD_writing_buffer[i++] = (uint8_t)(extension_time_ticks_buffer & 0xff); // LSB
	SD_writing_buffer[i++] = (uint8_t)((extension_time_ticks_buffer >> 8) & 0xff); // MSB
	buffer_busy = 0; // Opens the buffer again
	SD_writing_buffer[i++] = (uint8_t)INJ_exec_time_1; // LSB
	SD_writing_buffer[i++] = (uint8_t)INJ_exec_time_2; // LSB
	SD_writing_buffer[i++] = ADCmgr_binary_inputs_status_read(); // digital inputs status
    uint16_t CK_SUM = COMM_calculate_checksum(SD_writing_buffer, 0, i);
    SD_writing_buffer[i++] = (uint8_t)(CK_SUM >> 8);
    SD_writing_buffer[i++] = (uint8_t)(CK_SUM & 0xFF);

#if SD_MODULE_PRESENT
	// Writing the data on SD memory
	if (SD_init_OK == true){ // SD card initialized properly
	
		if (ADCmgr_battery_status_read() || bat_check_inhibit()){ // logs only if Battery is ON (or if battery check inhibit config flag is active)
			File dataFile = SD.open(file_name, FILE_WRITE); // Opens the file only if there is battery voltage
			if (dataFile){ // log only when the file is open, and when battery is connected (to avoid memory corruption)
				
				uint8_t data_bytes_written; // Data bytes effectively written on the SD card
				bool error_status = false; // Error status

				// Engine info
				if (!eng_log_inhibit()){
					data_bytes_written = dataFile.write(SD_writing_buffer, SD_WRITE_BUFFER_SIZE); // Writes engine related info (calculated above)
					if (data_bytes_written != SD_WRITE_BUFFER_SIZE) error_status = true; // not all bytes were written
				}
				
				// GPS info or Lambda info
				if ((GPS_SD_writing_request == true) && !gps_log_inhibit()) {
					data_bytes_written = dataFile.write(GPS_recv_buffer, GPS_SD_writing_request_size); // Write GPS data
					if (data_bytes_written != GPS_SD_writing_request_size) error_status = true; // not all bytes were written
					GPS_SD_writing_request = false; // reset flag (necessary to re-enable filling the buffer from GPS module)
				}else if ((ADCmgr_lambda_acq_buf_filled == true)  && !lam_log_inhibit()){ // Write LAMBDA info
					buffer_busy = 1; // Locks the buffer access (semaphore)
					ADCmgr_lambda_acq_buf[ADCMGR_LAMBDA_ACQ_BUF_TOT-4] = (uint8_t)(delta_inj_tick_buffer & 0xff); // LSB
					ADCmgr_lambda_acq_buf[ADCMGR_LAMBDA_ACQ_BUF_TOT-3] = (uint8_t)((delta_inj_tick_buffer >> 8) & 0xff); // MSB
					buffer_busy = 0; // Opens the buffer again
					CK_SUM = COMM_calculate_checksum((uint8_t*)ADCmgr_lambda_acq_buf, 0, (ADCMGR_LAMBDA_ACQ_BUF_TOT-2));
					ADCmgr_lambda_acq_buf[(ADCMGR_LAMBDA_ACQ_BUF_TOT-2)] = (uint8_t)(CK_SUM >> 8);
					ADCmgr_lambda_acq_buf[(ADCMGR_LAMBDA_ACQ_BUF_TOT-1)] = (uint8_t)(CK_SUM & 0xFF);
					data_bytes_written = dataFile.write((uint8_t*)ADCmgr_lambda_acq_buf, ADCMGR_LAMBDA_ACQ_BUF_TOT); // Write Lambda data
					if (data_bytes_written != ADCMGR_LAMBDA_ACQ_BUF_TOT) error_status = true; // not all bytes were written
					ADCmgr_lambda_acq_buf_filled = false; // reset Lambda writing flag, so the buffer can be filled in again if necessary
				}
				
				// IMU info (many packets accumulated in the buffer)
				if (!imu_log_inhibit()){
					uint8_t buffer_temporary[MPU6050_BUFFER_SD_WRITE_SIZE]; // create temporary writing buffer
					while (MPU6050mgr.prepare_SD_packet(buffer_temporary)){
						data_bytes_written = dataFile.write(buffer_temporary, MPU6050_BUFFER_SD_WRITE_SIZE); // Write IMU data
						if (data_bytes_written != MPU6050_BUFFER_SD_WRITE_SIZE) error_status = true; // not all bytes were written
					}
				}

				// Error status check
				if (error_status == true) { // Could not write completely all data
					write_errors_cnt++; // Increase error counter
				}else{ // Completely written all data
					write_errors_cnt = 0; // No error
				}
				
				if (ADCmgr_battery_status_read() || bat_check_inhibit()) { // Data written only in case battery voltage is present, or if bypass flag is active
					dataFile.close(); // Really write the data on the SD, only if battery voltage is present
					if (error_status == false) { // No error found
						temp_reply = true; // Data log considered completed successfully
					}
				}
				
			}else{ // Could not open the file
				write_errors_cnt++; // Increase error counter
			}
		}
		
		// Errors max check
		if (write_errors_cnt >= MAX_WRITE_ERRORS) {
			SD_init_OK = false; // This will cause the SD card to be re-initialized at next function call
			write_errors_cnt = 0; // Reset error counter
		}
		
	}
#endif
	
	return temp_reply;
	
}

SDmgr_class SDmgr;

#endif