#ifndef SDmgr_h
#define SDmgr_h

#define SD_WRITE_BUFFER_SIZE 23 // Size of buffer for SD writing (Engine data only)

class SDmgr_class
{
  public:
    String file_name; // SD file name where to log data
	bool SD_init_OK; // if SD card was initialized correctly
	uint8_t SD_writing_buffer[SD_WRITE_BUFFER_SIZE]; // buffer for SD writing
	uint8_t packet_cnt; // increasing counter
	uint8_t write_errors_cnt; // increases when there is a writing fault. After reaching the maximum, an SD re-init (begin) is done.
	
	SDmgr_class(); // Constructor
	bool begin(); // SD initialization
	bool log_SD_data(); // Logs information

};

extern SDmgr_class SDmgr;

#endif