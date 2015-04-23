#ifndef MARS_EncoderML
#define MARS_EncoderML
#include "Arduino.h"
class EncoderML
{
	// Class to handle EncoderML data for the 2015 UP MARS devices.
	
	public:
	EncoderML();
	static int getPosition();
	static int getRotationDirection();
	static int EncoderMLPINA;  // this pin needs to support interrupts
	static int EncoderMLPINB;  // no interrupt required	
	static int INTERRUPTID;
	// variables modified by interrupt handler must be declared as volatile
	static volatile int EncoderMLPosition;
	static volatile long interruptsReceived;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	static volatile short currentDirection; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	static long previousPosition;
	static void setupEncoderML();	
	static void onEncoderMLInterrupt();
};

#endif