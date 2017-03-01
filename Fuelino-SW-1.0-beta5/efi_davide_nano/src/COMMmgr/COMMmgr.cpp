#ifndef COMMmgr_cpp
#define COMMmgr_cpp

#include "../INJmgr/INJmgr.h"
#include "../EEPROMmgr/EEPROMmgr.h"
#include "../SDmgr/SDmgr.h"
#include "../ADCmgr/ADCmgr.h" // ADC manager. To have access to Analog readings
#include "../MPU6050mgr/MPU6050mgr.h" // IMU manager. To have access to IMU readings
#include "../compile_options.h"
#include "COMMmgr.h"

#define COMM_SERIAL_RECV_BYTES_NUM 8 // Buffer size for incoming data from SW and HW serial
#define COMM_SERIAL_STR_LEN_MAX 26 // Temporary buffer for string conversion (26 bytes should be enough for GPS config message)

uint8_t serial_inbyte_HW[COMM_SERIAL_RECV_BYTES_NUM]; // buffer for data received by Serial (HW seriale)
uint8_t serial_inbyte_SW[COMM_SERIAL_RECV_BYTES_NUM]; // buffer for data received by Serial (SW seriale)
uint8_t serial_byte_cnt_HW = 0; // contatore di numero bytes ricevuti in seriale
uint8_t serial_byte_cnt_SW = 0; // contatore di numero bytes ricevuti in seriale


// Initializes communication ports
void COMM_begin(){
	
#if (FUELINO_HW_VERSION >= 2) // Fuelino V2 has SWseriale pins (RX = 2, TX = 4)
	SWseriale.begin(); // Initializes Software seriale (Bluetooth module)
#endif

#if ENABLE_BUILT_IN_HW_SERIAL
	Serial.begin(57600); // Initializes PC serial (USB port)
#endif

}


// CALCULATED THE UBX CHECKSUM OF A CHAR ARRAY
uint16_t COMM_calculate_checksum(uint8_t* array, uint8_t array_start, uint8_t array_length) {
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	uint8_t i;
	for (i = 0; i < array_length; i++) {
		CK_A = CK_A + array[array_start + i];
		CK_B = CK_B + CK_A;
	}
	return ((CK_A << 8) | (CK_B));
}


// Adds checksum at the end of the array, and sends it via Serial, on SWseriale
void COMM_Send_Char_Array(COMM_destination_port_enum send_port, uint8_t* array_data, uint8_t array_size, bool checksum_enable){
		
	if (checksum_enable == true){
		uint16_t CK_SUM = COMM_calculate_checksum(array_data, 0, array_size);
		array_data[array_size] = CK_SUM >> 8; // CK_A
		array_data[array_size+1] = CK_SUM & 0xFF; // CK_B
		array_size += 2; // add checksum bytes
	}
		
	#if (FUELINO_HW_VERSION >= 2)
	if ((send_port == SW_SERIAL) || (send_port == ALL_SERIAL)) SWseriale.write(array_data, array_size); // Send complete string, including 2 bytes checksum
	#endif

	#if ENABLE_BUILT_IN_HW_SERIAL
	if ((send_port == HW_SERIAL) || (send_port == ALL_SERIAL)) Serial.write(array_data, array_size); // Send complete string, including 2 bytes checksum
	#endif
		
}


// Sends a string using SWseriale
void COMM_Send_String(COMM_destination_port_enum send_port, String input_string, bool end_line) {
	
	if (end_line == true){
		input_string+='\r';
		input_string+='\n';
	}
	uint8_t string_size = input_string.length();
	if (string_size > COMM_SERIAL_STR_LEN_MAX) string_size = COMM_SERIAL_STR_LEN_MAX;
	uint8_t serial_out_bytes[COMM_SERIAL_STR_LEN_MAX]; // stringa data fuori in output per il logger, via serial
	input_string.toCharArray((char*)serial_out_bytes, COMM_SERIAL_STR_LEN_MAX);
	COMM_Send_Char_Array(send_port, serial_out_bytes, string_size, false);
	
}


// Converts a character number (0-9) into an unsigned int number
uint8_t COMM_convert_char_array_to_num(uint8_t* input_array, uint8_t array_start, uint8_t array_len){
	uint16_t output_value_sum = 0;
	for (uint16_t i=0;i<array_len;i++){
		uint16_t cypher_tmp = input_array[array_start+i] - '0'; // transforms cypher into number
		if (cypher_tmp > 9) return 0x00; // not valid number
		for (uint8_t j=0; j<(array_len-i-1); j++){
			cypher_tmp*=10; // 100
		}
		output_value_sum += cypher_tmp;
	}
	if (output_value_sum > 255) return 0x00; // not valid number
	return (uint8_t)(output_value_sum & 0xFF); // converts the result in one byte
}


// Sends NACK
void COMM_send_nack(String nack_code, COMM_destination_port_enum send_port){
	COMM_Send_String(send_port, nack_code, true); // // NACK reply
}


// Sends back a service message to a tester
void COMM_send_service_message(String singleMessageString, COMM_destination_port_enum send_port){
		COMM_Send_String(send_port, singleMessageString, true); //send string to destination port
}


// Evaluates the command received from Serial Port
uint8_t COMM_evaluate_parameter_read_writing_request(uint8_t* data_array, uint8_t data_size, COMM_destination_port_enum recv_port){

	String singleMessageString = ""; // Reply message initialization
	
	// EEPROM command
	if ((data_array[0] == 'e') && (data_size == 8)){ // EEPROM command (e w xxx yyy)
		uint8_t read_write = COMM_convert_char_array_to_num(data_array, 1, 1);
		uint8_t sum_address = COMM_convert_char_array_to_num(data_array, 2, 3);
		uint8_t sum_value = COMM_convert_char_array_to_num(data_array, 5, 3);
		if ((read_write==0) && (sum_value == 0)){ // Read data
			sum_value = EEPROM.read(sum_address);
		}
		else if (read_write==1){ // Write data
			EEPROM.write(sum_address, sum_value);
		}
		else{ // Not proper format, send NACK response
			COMM_send_nack(F("e9999999"), recv_port); // // NACK reply
			return 0; //  NG
		}
		singleMessageString += F("e"); // (Header)
		singleMessageString += read_write; // Read (0) or Write (1)
		if (sum_address < 10) singleMessageString += F("0"); // 3 cyphers
		if (sum_address < 100) singleMessageString += F("0"); // 3 cyphers
		singleMessageString += sum_address; // 3 cyphers
		if (sum_value < 10) singleMessageString += F("0"); // 3 cyphers
		if (sum_value < 100) singleMessageString += F("0"); // 3 cyphers
		singleMessageString += sum_value; // 3 cyphers
		COMM_send_service_message(singleMessageString, recv_port); //send string
		return 1; // OK
	}
	
	// CALIBRATION command
	else if ((data_array[0] == 'c') && (data_size == 8)){
		uint8_t read_write = COMM_convert_char_array_to_num(data_array, 1, 1);
		uint8_t map_num = COMM_convert_char_array_to_num(data_array, 2, 1);
		uint8_t sum_index = COMM_convert_char_array_to_num(data_array, 3, 2);
		uint8_t sum_value = COMM_convert_char_array_to_num(data_array, 5, 3);
		singleMessageString += F("c"); // (Header)
		singleMessageString += read_write; // Read (0) or Write (1)
		singleMessageString += map_num; // Map number
		if ((read_write==0) && (sum_value == 0)){ // Read from RAM
			sum_value = 0xFF; // tentative value (error case)
			if (sum_index < 10) singleMessageString += F("0"); // Index
			singleMessageString += sum_index; // Index
			if ((map_num == 0) && (sum_index < INJ_INCR_RPM_MAPS_SIZE)){
				sum_value = incrementi_rpm[sum_index];
			}
			else if ((map_num == 1) && (sum_index < INJ_INCR_THR_MAPS_SIZE)){
				sum_value = incrementi_thr[sum_index];
			}else{
				COMM_send_nack(F("c0999999"), recv_port); // NACK reply
				return 0; // NG
			}
			if (sum_value < 10) singleMessageString += F("0"); // 3 cyphers
			if (sum_value < 100) singleMessageString += F("0"); // 3 cyphers
			singleMessageString += sum_value; // 3 cyphers
			COMM_send_service_message(singleMessageString, recv_port); //send string
			return 1; // OK
		}
		else if ((read_write==1)){ // Write to RAM
			if ((map_num == 0) && (sum_index < INJ_INCR_RPM_MAPS_SIZE)){
				incrementi_rpm[sum_index]=sum_value; // write the value in RAM
				sum_value=incrementi_rpm[sum_index]; // check back
			}
			else if ((map_num == 1) && (sum_index < INJ_INCR_THR_MAPS_SIZE)){
				incrementi_thr[sum_index]=sum_value; // write the value in RAM
				sum_value=incrementi_thr[sum_index]; // check back
			}else{
				COMM_send_nack(F("c1999999"), recv_port); // // NACK reply
				return 0; // NG
			}
			if (sum_index < 10) singleMessageString += F("0"); // Index
			singleMessageString += sum_index; // Index
			if (sum_value < 10) singleMessageString += F("0"); // 3 cyphers
			if (sum_value < 100) singleMessageString += F("0"); // 3 cyphers
			singleMessageString += sum_value; // 3 cyphers
			COMM_send_service_message(singleMessageString, recv_port); //send string
			return 1; // OK
		}
		else if ((read_write==2) && (map_num<=INJ_MAPS_TOTAL_NUM) && (sum_index == 0)){ // Write to EEPROM
			singleMessageString += F("0000"); // 00 00
			if (sum_value == 0){
				EEPROM_write_standard_values(map_num); // c n 2 00 000
				singleMessageString += F("0"); // 0
			}else if (sum_value == 1){
				EEPROM_write_RAM_map_to_EEPROM(map_num); // c n 2 00 001
				singleMessageString += F("1"); // 1
			}else{
				COMM_send_nack(F("c2999999"), recv_port); // // NACK reply
				return 0; // NG
			}
			COMM_send_service_message(singleMessageString, recv_port); //send string
			return 1; // OK
		}
		else{
			COMM_send_nack(F("c9999999"), recv_port); // // NACK reply
			return 0; // Error
		}
	}
	
	// DATA command
	else if ((data_array[0] == 'd') && (data_size >= 4) && (data_size <= 8)){ // DATA command (d x yy ...)
		if (data_array[1] == '0'){ // d 0 ... (ASCII request)
			uint8_t request_num = COMM_convert_char_array_to_num(data_array, 2, 2);
			uint16_t val_to_send;
			bool req_good = true;
			singleMessageString += F("d0"); // (Header) d 0
			if (request_num < 10) singleMessageString += F("0"); // 2 cyphers
			singleMessageString += request_num; // 2 cyphers
			buffer_busy = 1; // Locks the buffer access (semaphore)
			if (request_num == 0){ // d 0 0 0 ... // 2rpm
				val_to_send = delta_time_tick_buffer;
			}
			else if (request_num == 1){ // d 0 0 1 ... // inj time
				val_to_send = delta_inj_tick_buffer;
			}
			else if (request_num == 2){ // d 0 0 2 ... // extension time
				val_to_send = extension_time_ticks_buffer;
			}
			else if (request_num == 3){ // d 0 0 3 ... // throttle
				val_to_send = throttle_buffer;
			}
			else if (request_num == 4){ // d 0 0 4 ... // lambda
				val_to_send = lambda_buffer;
			}
			else if (request_num == 5){ // d 0 0 5 ... // combustion counter
				val_to_send = injection_counter_buffer;
			}
			else if (request_num == 6){ // d 0 0 6 ... // combustion counter, and activate one Lambda sensor screenshot
				INJmgr.steady_state_prescaler_tgt = 3; // 3 * 1085 us * 32 = about 100ms, therefore 1 cycle is about 2 rotations at 1200rpm
				val_to_send = injection_counter_buffer;
			}
			else if (request_num == 7){ // d 0 0 7 ... // digital inputs status
				val_to_send = ADCmgr_binary_inputs_status_read();
			}
			else{
				req_good = false; // no valid request
			}
			buffer_busy = 0; // Opens the buffer again
			if (req_good == true){
				if (val_to_send < 10) singleMessageString += F("0"); // 5 cyphers
				if (val_to_send < 100) singleMessageString += F("0"); // 5 cyphers
				if (val_to_send < 1000) singleMessageString += F("0"); // 5 cyphers
				if (val_to_send < 10000) singleMessageString += F("0"); // 5 cyphers
				singleMessageString += val_to_send; // 5 cyphers
				COMM_send_service_message(singleMessageString, recv_port); //send string
				return 1; // OK
			}
		}
		else if (data_array[1] == '1'){ // d 1 ... (binary request)
			if ((data_array[2] == '0') && (data_array[3] == '0')){ // d 1 0 0 ... (binary request) -> Engine data
				COMM_Send_Char_Array(recv_port, SDmgr.SD_writing_buffer, SD_WRITE_BUFFER_SIZE, false);
				return 1; // OK
			}
			else if ((data_array[2] == '0') && (data_array[3] == '1')){ // d 1 0 1 ... (binary request) -> IMU data
				uint8_t IMU_buffer_COMM_tmp[(MPU6050_BUFFER_COMM_SIZE + 2)]; // allocates buffer (data + checksum)
				MPU6050mgr.prepare_COMM_packet(IMU_buffer_COMM_tmp); // prepares buffer (data)
				COMM_Send_Char_Array(recv_port, IMU_buffer_COMM_tmp, MPU6050_BUFFER_COMM_SIZE, true); // attach checksum and send packet
				return 1; // OK
			}
		}
		else if ((data_array[1] == 'i') && (data_array[2] == 'm') && (data_array[3] == 'u')){ // d i m u ... (ASCII request) -> IMU data for calibration
			MPU6050mgr.send_ASCII_data();
			return 1; // OK
		}
		COMM_send_nack(F("d9999999"), recv_port); // // NACK reply
		return 0;
	}
	
	COMM_send_nack(F("99999999"), recv_port); // // NACK reply
	return 0; // Error
}


// Checks data coming from Serial connections
void COMM_receive_check(){
	
#if (FUELINO_HW_VERSION >= 2) // Fuelino V2 has SWseriale pins (RX = 2, TX = 4)
	#if (BLUETOOTH_PRESENT) // Bluetooth gateway
	while (SWseriale.available()) { // carattere ricevuto
		uint8_t temp_char_read = SWseriale.read();
		if ((temp_char_read == '\n') || (temp_char_read == '\r')){ // End of command
			if ((serial_byte_cnt_SW != 0) && ((serial_inbyte_SW[0] == 'e') || (serial_inbyte_SW[0] == 'c') || (serial_inbyte_SW[0] == 'd'))) COMM_evaluate_parameter_read_writing_request(serial_inbyte_SW, serial_byte_cnt_SW, SW_SERIAL);
			serial_byte_cnt_SW = 0; // restart from zero
		}
		else{ // Store character into buffer
			if (serial_byte_cnt_SW == COMM_SERIAL_RECV_BYTES_NUM) serial_byte_cnt_SW = 0; // buffer is full
			serial_inbyte_SW[serial_byte_cnt_SW] = temp_char_read;
			serial_byte_cnt_SW++;
		}
		//delay(2); // Give time to the next char to arrive
	}
	#endif
#endif

#if ENABLE_BUILT_IN_HW_SERIAL	
	while (Serial.available()) { // carattere ricevuto
		uint8_t temp_char_read = Serial.read();
		if ((temp_char_read == '\n') || (temp_char_read == '\r')){ // End of command
			if ((serial_byte_cnt_HW != 0) && ((serial_inbyte_HW[0] == 'e') || (serial_inbyte_HW[0] == 'c') || (serial_inbyte_HW[0] == 'd'))) COMM_evaluate_parameter_read_writing_request(serial_inbyte_HW, serial_byte_cnt_HW, HW_SERIAL);
			serial_byte_cnt_HW = 0; // restart from zero
		}
		else{ // Store character into buffer
			if (serial_byte_cnt_HW == COMM_SERIAL_RECV_BYTES_NUM) serial_byte_cnt_HW = 0; // buffer is full
			serial_inbyte_HW[serial_byte_cnt_HW] = temp_char_read;
			serial_byte_cnt_HW++;
		}
	}
#endif
	
}

#endif