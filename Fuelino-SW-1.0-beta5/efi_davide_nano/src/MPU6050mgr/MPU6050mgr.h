#ifndef MPU6050mgr_h
#define MPU6050mgr_h

#define MPU6050_BUFFER_IMU_SIZE (uint8_t)12 // IMU data buffer size
#define MPU6050_BUFFERS_NUMBER (uint8_t)4 // Number of buffers for IMU data
#define MPU6050_BUFFER_SD_WRITE_SIZE (uint8_t)(1 + 4 + MPU6050_BUFFER_IMU_SIZE + 2 + 2) // Size of buffer for SD write (header + time + data + temperature + checksum) | 21 bytes
#define MPU6050_BUFFER_COMM_SIZE 14 // packet for COMM Service via Serial

typedef struct{
	uint8_t polling_time_stamp[4]; // Time stamp of polling (32 bit data)
	uint8_t IMU_data[MPU6050_BUFFER_IMU_SIZE]; // IMU data received by MPU6050
}data_buffer_struct;

class MPU6050mgr_class{
	
	public:
		void begin(); // Initialization function
		void flush_buffer();
		void manager(unsigned long time_now_ms); // Main manager
		int16_t ag[6]; // Acceleration and gyroscope data
		int16_t temperature; // temperature received from MPU6050 (2 bytes)
		uint8_t buffer_data_available(); // returns the number of buffer packets available to write on SD
		uint8_t prepare_SD_packet(uint8_t* temp_data_buffer_SD);
		void prepare_COMM_packet(uint8_t* temp_data_buffer_COMM);
		void send_ASCII_data();
		data_buffer_struct data_buffer[MPU6050_BUFFERS_NUMBER];
		
	private:
		//uint8_t next_buffer_index();
		int16_t filter_signal(int16_t sig_old, int16_t sig_new); // Filters acceleration or gyroscope signal, and returns the filtered value
		uint8_t read_data(); // Reads data from MPU6050, filters it, and saves it onto the buffer
		uint16_t polling_time_last = 0; // Last time the polling has been done
		uint8_t task_multiplier_cnt = 0; // Multiplier
		uint8_t item_last_read = 0; // Read from SD module, and written to SD card
		uint8_t item_last_write = 0; // Filled in by MPU6050 module
		
};

extern MPU6050mgr_class MPU6050mgr;

#endif