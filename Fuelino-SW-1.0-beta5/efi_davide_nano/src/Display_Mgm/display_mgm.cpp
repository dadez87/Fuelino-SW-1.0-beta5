#ifndef DISPLAY_MGR_cpp
#define DISPLAY_MGR_cpp

#include "display_mgm.h"

Display_Mgr Display;

Display_Mgr::Display_Mgr()
{
	display_page_sts = NO_PAGE; // init page
	req_cycle_counter = 0;
}

// Evaluates if needed to update the Display or not
void Display_Mgr::DisplayManager(){
#if DISPLAY_PRESENT == 1
	req_cycle_counter++;
	if (req_cycle_counter == CYCLE_COUNTER_MAX_PLOT){
		buffer_data(); // bufferizes data
		page_reflash(); // reflash page
		req_cycle_counter = 0;
	}
#endif
}

// Prints the standard message on LCD screen
void Display_Mgr::initialize(){
#if DISPLAY_PRESENT == 1
	lcd.begin(8, 2); 
	lcd.setContrast(10);
	lcd.setCursor(0, 0); 
	lcd.print("Fuelino ");
	lcd.setCursor(0, 1); 
	lcd.print("SW V1 b1");
#endif
}

// Bufferizes data before plotting it on the screen
void Display_Mgr::buffer_data(){
	
}

// CREATES THE "ECU Fuel Injection Time" PAGE
void Display_Mgr::ECU_FIT_LINE_creator(){
	display_char_line1+="InjTime";
	//if (delta_inj_tick_buffer<10) display_char_line1+=' ';
	//if (delta_inj_tick_buffer<100) display_char_line1+=' ';
	//if (delta_inj_tick_buffer<1000) display_char_line1+=' ';
	//if (delta_inj_tick_buffer<10000) display_char_line1+=' ';
	display_char_line2+=((uint32_t)delta_inj_tick_buffer << 2);
	display_char_line2+="u";
}
	
// CREATES THE "ECU RPM" PAGE
void Display_Mgr::ECU_RPM_LINE_creator(){
	display_char_line1+="DeltaTim";
	display_char_line2+=((uint32_t)delta_time_tick_buffer << 2);
	display_char_line2+="u";
}

// CREATES THE "ECU Throttle" PAGE
void Display_Mgr::ECU_THR_LINE_creator(){
	display_char_line1+="Throttle";
	display_char_line2+=throttle_buffer;
	//display_char_line2+="";
}

// REFLASHES THE DISPLAY PAGE
void Display_Mgr::page_reflash(){
	
	// Create lines contents
	display_char_line1="";
	display_char_line2="";
	buffer_busy = 1; // Locks the buffer access (semaphore)
	switch (display_page_sts) {
		case NO_PAGE:
			display_char_line1+="Init";
			display_page_sts = ECU_FIT_PAGE; // increase page
			break;

		case ECU_FIT_PAGE:
			ECU_FIT_LINE_creator();
			display_page_sts = ECU_RPM_PAGE; // increase page
			break;
		
		case ECU_RPM_PAGE:
			ECU_RPM_LINE_creator();
			display_page_sts = ECU_THR_PAGE; // increase page
			break;
			
		case ECU_THR_PAGE:
			ECU_THR_LINE_creator();
			display_page_sts = ECU_FIT_PAGE; // increase page
			break;
	}
	buffer_busy = 0; // Opens the buffer again
	
	// Print lines on the LCD
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(display_char_line1);
	lcd.setCursor(0, 1);
	lcd.print(display_char_line2);
}

#endif
