#ifndef MARS_ENCODER
#define MARS_ENCODER
#include "Arduino.h"
class Encoder
{
	// Class to handle encoder data for the 2015 UP MARS devices.
	
	public:
	Encoder(int ENCODER0PINA_in, int  ENCODER0PINB_in, int interruptID);
	long getPosition();
	int getRotationDirection();
	
	private:
	int ENCODER0PINA = 0;  // this pin needs to support interrupts
	int ENCODER0PINB = 0;  // no interrupt required	
	// variables modified by interrupt handler must be declared as volatile
	volatile long encoder0Position = 0;
	volatile long interruptsReceived = 0;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	volatile short currentDirection = -1; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	long previousPosition = 0;
	void setupEncoder(int interruptID);
};

void onInterrupt();
#endif