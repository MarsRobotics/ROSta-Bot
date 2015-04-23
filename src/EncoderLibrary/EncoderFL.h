#ifndef MARS_EncoderFL
#define MARS_EncoderFL
#include "Arduino.h"
class EncoderFL
{
	// Class to handle EncoderFL data for the 2015 UP MARS devices.
	
	public:
	EncoderFL();
	static int getPosition();
	static int getRotationDirection();
	static int EncoderFLPINA;  // this pin needs to support interrupts
	static int EncoderFLPINB;  // no interrupt required	
	static int INTERRUPTID;
	// variables modified by interrupt handler must be declared as volatile
	static volatile int EncoderFLPosition;
	static volatile long interruptsReceived;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	static volatile short currentDirection; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	static long previousPosition;
	static void setupEncoderFL();	
	static void onEncoderFLInterrupt();
};

#endif