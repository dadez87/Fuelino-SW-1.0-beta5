#ifndef COMMmgr_h
#define COMMmgr_h

#include <Arduino.h>
#include "SWSeriale/SWseriale.h" // SW serial using INT1 and Timer2

enum COMM_destination_port_enum{
	HW_SERIAL = 0, // 
	SW_SERIAL, // 
	ALL_SERIAL // 
};

// Global variables to be exported

// Functions to be exported
extern void COMM_begin();
extern uint16_t COMM_calculate_checksum(uint8_t* array, uint8_t array_start, uint8_t array_length);
extern void COMM_Send_Char_Array(COMM_destination_port_enum send_port, uint8_t* array_data, uint8_t array_size, bool checksum_enable);
extern void COMM_Send_String(COMM_destination_port_enum send_port, String input_string, bool end_line);
extern void COMM_receive_check();

#endif