#ifndef GPSmgr_h
#define GPSmgr_h

#include "../COMMmgr/COMMmgr.h" // for Serial Communication

#define GPS_SEND_BUFFER_SIZE 8
#define GPS_RECV_BUFFER_SIZE 36

extern void GPS_initialize();
extern void GPS_manager();

extern uint8_t GPS_recv_buffer[GPS_RECV_BUFFER_SIZE]; // GPS received data, for SD logging
extern bool GPS_SD_writing_request; // writing request to SD module, flag
extern uint8_t GPS_SD_writing_request_size; // writing request to SD module, size

// Export for SD file date
extern uint16_t GPS_year;
extern uint8_t GPS_month;
extern uint8_t GPS_day;
extern uint8_t GPS_hour;
extern uint8_t GPS_min;
extern uint8_t GPS_sec;

#endif