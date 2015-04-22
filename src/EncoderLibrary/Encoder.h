#ifndef MARS_ENCODER
#define MARS_ENCODER
#include "Arduino.h"
class Encoder
{
	// Class to handle encoder data for the 2015 UP MARS devices.
	
	public:
	Encoder();
	static int getPosition();
	static int getRotationDirection();
	static int ENCODERPINA;  // this pin needs to support interrupts
	static int ENCODERPINB;  // no interrupt required	
	static int INTERRUPTID;
	// variables modified by interrupt handler must be declared as volatile
	static volatile int encoderPosition;
	static volatile long interruptsReceived;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	static volatile short currentDirection; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	static long previousPosition;
	static void setupEncoder();	
	static void onEncoderInterrupt();
};

#endif