#ifndef TEMPO_h
#define TEMPO_h

#include <avr/io.h>
#include <avr/interrupt.h>

class Tempo
{
  public:
  
    // properties
    unsigned char clockSelectBits; // 
	char oldSREG; // To hold Status Register while ints disabled

    // methods
    void initialize();
    void start(); // starts the timer
	void reset(); // resets the timer
    void stop(); // stops the timer
	uint16_t read(); // returns the time of the counter
    void attachInterrupt(void (*isr)());
    void detachInterrupt();
    void setPeriod(uint16_t ticks_num);
	void (*isrCallback)();
};

extern Tempo Timer1;
#endif