#ifndef TEMPO_cpp
#define TEMPO_cpp

#include "tempo.h"

Tempo Timer1;              // preinstatiate

ISR(TIMER1_COMPA_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  Timer1.isrCallback();
}

void Tempo::initialize()
{
  TCCR1A = 0; // clear control register A 
  TCCR1B = _BV(WGM12); // turn on CTC mode
  clockSelectBits = _BV(CS11); // prescale by /8 = 0.5us at 16MHz
}

void Tempo::setPeriod(uint16_t ticks_num) // sets the period (in ticks), 1 tick = 0.5 us
{
  oldSREG = SREG;				
  cli(); // Disable interrupts for 16 bit register access
  OCR1A = ticks_num; // 2 bytes write
  SREG = oldSREG;
}


// Attaches an interrupt to the Timer1 interrupt event
void Tempo::attachInterrupt(void (*isr)())
{
  isrCallback = isr; // register the user's callback with the real ISR
  TIMSK1 = _BV(OCIE1A); // sets the timer overflow interrupt enable bit on compare
  //sei(); //might be running with interrupts disabled (eg inside an ISR), so don't touch the global state
}


// Removes the Timer1 interrupt
void Tempo::detachInterrupt()
{
  TIMSK1 &= ~_BV(OCIE1A); // clears the timer overflow interrupt enable bit, timer continues to count without calling the isr 
}


// Starts the timer
void Tempo::start()
{ 
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits; // fa partire il timer
}


// Stops the timer
void Tempo::stop()
{
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12)); // clears all clock selects bits
}


// Resets the counter
void Tempo::reset()
{ 
	oldSREG= SREG;
  	cli();	
	TCNT1H=0;
	TCNT1L=0;
	SREG = oldSREG;
}


// Return the number of Timer1 ticks
uint16_t Tempo::read(){				
	oldSREG= SREG;
  	cli();							
  	uint16_t tmp=TCNT1;    					
	SREG = oldSREG;
	return tmp;
}

#endif