#ifndef GPSmgr_cpp
#define GPSmgr_cpp

#include "GPSmgr.h"

// Info about polling strategy
#define GPS_MGR_DELAY_SEND_AFTER_RECV (uint16_t)500 // Delay of ms after receiving, to send the new polling message [ms]
#define GPS_MGR_DELAY_SEND_AFTER_SEND (uint16_t)700 // Delay of ms after sending, to send the new polling message [ms]

// Info about buffers and protocol bytes
#define UBX_NAV_POSLLH_BYTE 0x02
#define UBX_NAV_TIMEGPS_BYTE 0x20
#define UBX_NAV_TIMEUTC_BYTE 0x21
#define UBX_NAV_POSLLH_CHARS 36
#define UBX_NAV_TIMEGPS_CHARS 24
#define UBX_NAV_TIMEUTC_CHARS 28

// Info about how to determine how to switch from time to position mode
#define UBX_NAV_TIMEGPS_OK_MAX 5 // To determine when to swith to POSLLH packets
#define UBX_NAX_TIMEGPS_RECV_MAX_ENABLE 0 // Enables switching to POSLLH when crossing the max
#define UBX_NAV_TIMEGPS_RECV_MAX 100 // To determine when to swith to POSLLH packets

#define GPS_PACKET_FORWARD_DEBUG_ENABLE 0 // Enables forwarding of GPS packets, to HW Serial

uint8_t GPS_recv_mode_TIMEGPS_OK_cnt = 0;
uint8_t GPS_recv_mode_TIMEGPS_recv_cnt = 0;

//uint8_t GPS_manager_task_cnt = 0;
uint8_t GPS_send_buffer[GPS_SEND_BUFFER_SIZE]; // send char buffer
uint8_t GPS_recv_buffer[GPS_RECV_BUFFER_SIZE]; // GPS received data
uint8_t GPS_recv_buffer_cnt = 0; // counter for chars received by GPS
bool UBX_NAV_mess_recv_started = false; // message receiving has started
bool GPS_SD_writing_request = false; // flag to request SD card writing
uint8_t GPS_SD_writing_request_size = 0; // number of bytes to be written to SD card
bool GPS_msg_sent_not_yet_recv = false; // this flag becomes "true" after sending, and remains true until a complete reception happens, or until there is an error

uint16_t GPS_last_time_sent = 0; // last time in which a polling message was sent to GPS
uint16_t GPS_last_time_recv = 0; // last time in which a message was received by GPS

// Date for SD logging
uint16_t GPS_year = 1987;
uint8_t GPS_month = 11;
uint8_t GPS_day = 6;
uint8_t GPS_hour = 8;
uint8_t GPS_min = 0;
uint8_t GPS_sec = 0;


// SENDS (UBX) POLLING MESSAGE
void GPS_UBX_NAV_polling_preparation(uint8_t UBX_NAV_code){
	GPS_send_buffer[0]=0xB5; // Header UBX
	GPS_send_buffer[1]=0x62; // Header UBX
	GPS_send_buffer[2]=0x01; // Class UBX NAV
	GPS_send_buffer[3]=UBX_NAV_code; // ID
	GPS_send_buffer[4]=0x00; // Length
	GPS_send_buffer[5]=0x00; // Length
	uint16_t CK_SUM = COMM_calculate_checksum(GPS_send_buffer, 2, 4);
	GPS_send_buffer[6] = CK_SUM >> 8; // CK_A
	GPS_send_buffer[7] = CK_SUM & 0xFF; // CK_B
}


// GPS Initialization. Disables messages from GPS module.
void GPS_initialize(){
    delay(1000);
    COMM_Send_String(SW_SERIAL, F("$PUBX,40,RMC,0,0,0,0*47"), true); //RMC OFF
    delay(100);
    COMM_Send_String(SW_SERIAL, F("$PUBX,40,VTG,0,0,0,0*5E"), true); //VTG OFF
    delay(100);
    COMM_Send_String(SW_SERIAL, F("$PUBX,40,GGA,0,0,0,0*5A"), true); //CGA OFF
    delay(100);
    COMM_Send_String(SW_SERIAL, F("$PUBX,40,GSA,0,0,0,0*4E"), true); //GSA OFF
    delay(100);
    COMM_Send_String(SW_SERIAL, F("$PUBX,40,GSV,0,0,0,0*59"), true); //GSV OFF
    delay(100);
    COMM_Send_String(SW_SERIAL, F("$PUBX,40,GLL,0,0,0,0*5C"), true); //GLL OFF
    delay(100);
	
	GPS_UBX_NAV_polling_preparation(UBX_NAV_TIMEUTC_BYTE); // prepares GPS polling message data
}


// Evaluates GPS mode change
void GPS_mode_change_evaluation(){
	if (GPS_recv_buffer[3] == UBX_NAV_TIMEUTC_BYTE){
		uint16_t CK_SUM = COMM_calculate_checksum(GPS_recv_buffer, 2, 24);
		uint8_t Ck_A = CK_SUM >> 8; // CK_A
		uint8_t Ck_B = CK_SUM & 0xFF; // CK_B
		if ((GPS_recv_buffer[26] == Ck_A) && (GPS_recv_buffer[27] == Ck_B)){ // Checkum check is correct
			uint8_t validity_flag = (GPS_recv_buffer[25] & 0x07); // Validity Flags first 3 bits
			//uint8_t tAcc_byte2 = GPS_recv_buffer[12]; // tAcc[2]
			//uint8_t tAcc_byte3 = GPS_recv_buffer[13]; // tAcc[3]
			if (validity_flag >= 0x03){ // Date and time reliability entry conditions
				GPS_year = (uint16_t)GPS_recv_buffer[18] | ((uint16_t)GPS_recv_buffer[19] << 8);
				GPS_month = GPS_recv_buffer[20];
				GPS_day = GPS_recv_buffer[21];
				GPS_hour = GPS_recv_buffer[22];
				GPS_min = GPS_recv_buffer[23];
				GPS_sec = GPS_recv_buffer[24];
			//}
			//if ((validity_flag == 0x07)){ // && (tAcc_byte2 == 0) && (tAcc_byte3 == 0)){ // Validity Flags, and time accuracy check, for mode change
				GPS_recv_mode_TIMEGPS_OK_cnt++; // OK. Increase OK counter
			}else{
				GPS_recv_mode_TIMEGPS_OK_cnt = 0; // Not OK. Reset counter
			}
			if (UBX_NAX_TIMEGPS_RECV_MAX_ENABLE) GPS_recv_mode_TIMEGPS_recv_cnt++; // TIME GPS recv message counter
			if ((GPS_recv_mode_TIMEGPS_OK_cnt >= UBX_NAV_TIMEGPS_OK_MAX) ||    // TIME GPS received was OK for many times
			   (GPS_recv_mode_TIMEGPS_recv_cnt >= UBX_NAV_TIMEGPS_RECV_MAX)){  // TIME GPS was received for many times
				GPS_UBX_NAV_polling_preparation(UBX_NAV_POSLLH_BYTE); // prepares GPS polling message data
			}
		}
	}
}


// Checks for any message coming from GPS, and sends polling request if necessary
void GPS_manager(){
	
	// Checks for received message from GPS
	while (SWseriale.available()) { // character has been received
		uint8_t temp_recv = SWseriale.read();
		bool reset_condition_request = false;
		if ((UBX_NAV_mess_recv_started == false) && (temp_recv == 0xB5) && (GPS_SD_writing_request == false) && (GPS_msg_sent_not_yet_recv == true)){ // UBX NAV receiving start conditions
			UBX_NAV_mess_recv_started = true; // starts receiving
			GPS_recv_buffer_cnt = 0; // received chars counter set to zero
		}
		if (UBX_NAV_mess_recv_started == true){ // message already started
			if ((GPS_recv_buffer_cnt == 1) && (temp_recv != 0x62)) reset_condition_request = true; // strange UBX NAV
			if ((GPS_recv_buffer_cnt == 2) && (temp_recv != 0x01)) reset_condition_request = true; // strange UBX NAV
			if ((GPS_recv_buffer_cnt == 3) && (temp_recv != UBX_NAV_POSLLH_BYTE) && (temp_recv != UBX_NAV_TIMEGPS_BYTE) && (temp_recv != UBX_NAV_TIMEUTC_BYTE)) reset_condition_request = true; // UBX NAV not supported
			GPS_recv_buffer[GPS_recv_buffer_cnt] = temp_recv; // save temporary char into the received message buffer
			GPS_recv_buffer_cnt++; // increase received chars counter
			if (((GPS_recv_buffer_cnt == UBX_NAV_POSLLH_CHARS) && (GPS_recv_buffer[3] == UBX_NAV_POSLLH_BYTE)) || // NAV_POSLLH received completely
			   ((GPS_recv_buffer_cnt == UBX_NAV_TIMEGPS_CHARS) && (GPS_recv_buffer[3] == UBX_NAV_TIMEGPS_BYTE)) || // NAV_TIMEGPS received completely
			   ((GPS_recv_buffer_cnt == UBX_NAV_TIMEUTC_CHARS) && (GPS_recv_buffer[3] == UBX_NAV_TIMEUTC_BYTE))) { // NAV_TIMEUTC received completely
				GPS_last_time_recv = (uint16_t)millis(); // read present time
				GPS_mode_change_evaluation(); // checks for GPS mode change
				GPS_SD_writing_request_size = GPS_recv_buffer_cnt; // SD writing request bytes number
				GPS_SD_writing_request = true; // SD writing request flag activation
				reset_condition_request = true;
				if (GPS_PACKET_FORWARD_DEBUG_ENABLE) COMM_Send_Char_Array(HW_SERIAL, GPS_recv_buffer, GPS_recv_buffer_cnt, false); // sends data to PC, for debugging
			}
			if ((GPS_recv_buffer_cnt == GPS_RECV_BUFFER_SIZE)){ // buffer overflow
				reset_condition_request = true;
			}
		}
		/*else{ // message not yet started
			reset_condition_request = true;
		}*/
		if (reset_condition_request == true){
			GPS_recv_buffer_cnt = 0; // no char received yet
			UBX_NAV_mess_recv_started = false; // message reception has not started
			GPS_msg_sent_not_yet_recv = false; // before listening again, a new message has to be received. At that time, this flag becomes "true"
		}
	}
	
	// Determines if a polling message has to be sent to the GPS unit
	uint16_t time_now_tmp = (uint16_t)millis(); // read present time
	if (((time_now_tmp - GPS_last_time_recv) >= GPS_MGR_DELAY_SEND_AFTER_RECV) && // Enough time has expired since the last time the message has been received
		((time_now_tmp - GPS_last_time_sent) >= GPS_MGR_DELAY_SEND_AFTER_SEND)){ // Enough time has expired since the last time the message has been sent
		GPS_last_time_sent = time_now_tmp; // time stamp is updated
		SWseriale.write(GPS_send_buffer, GPS_SEND_BUFFER_SIZE); // Send polling request
		GPS_msg_sent_not_yet_recv = true; // polling request has been sent, now it is time to receive the message
	}
	
}

#endif