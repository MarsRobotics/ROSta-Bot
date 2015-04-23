#ifndef MARS_EncoderRL
#define MARS_EncoderRL
#include "Arduino.h"
class EncoderRL
{
	// Class to handle EncoderRL data for the 2015 UP MARS devices.
	
	public:
	EncoderRL();
	static int getPosition();
	static int getRotationDirection();
	static int EncoderRLPINA;  // this pin needs to support interrupts
	static int EncoderRLPINB;  // no interrupt required	
	static int INTERRUPTID;
	// variables modified by interrupt handler must be declared as volatile
	static volatile int EncoderRLPosition;
	static volatile long interruptsReceived;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	static volatile short currentDirection; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	static long previousPosition;
	static void setupEncoderRL();	
	static void onEncoderRLInterrupt();
};

#endif