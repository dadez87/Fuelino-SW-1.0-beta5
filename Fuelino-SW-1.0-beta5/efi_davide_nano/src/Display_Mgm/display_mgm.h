#ifndef DISPLAY_MGR_h
#define DISPLAY_MGR_h

#include <Wire.h> 
#include "ST7032.h" // Library to manage display hardware
#include "../compile_options.h"
#include "../INJmgr/INJmgr.h" // INJ manager. To have access to variables (inj time, ...)

#define CYCLE_COUNTER_MAX_PLOT 10 // 10*100ms

// PAGES WHICH CAN BE VISUALIZED
enum display_page_sts_type{
	NO_PAGE = 0,
	ECU_FIT_PAGE,
	ECU_RPM_PAGE,
	ECU_THR_PAGE
};

class Display_Mgr{

	public:
	Display_Mgr();
	void initialize();
	void DisplayManager();

	private:
	ST7032 lcd;
	display_page_sts_type display_page_sts;
	uint8_t req_cycle_counter;
	String display_char_line1; // characters to be printed on the display (8 characters)
	String display_char_line2; // characters to be printed on the display (8 characters)
	void buffer_data(); // Creates the data buffer
	void ECU_FIT_LINE_creator(); // Fuel Injection time
	void ECU_RPM_LINE_creator(); // RPM
	void ECU_THR_LINE_creator(); // Throttle voltage
	void page_reflash();
	
};

extern Display_Mgr Display;

#endif