#ifndef MARS_EncoderMR
#define MARS_EncoderMR
#include "Arduino.h"
class EncoderMR
{
	// Class to handle EncoderMR data for the 2015 UP MARS devices.
	
	public:
	EncoderMR();
	static int getPosition();
	static int getRotationDirection();
	static int EncoderMRPINA;  // this pin needs to support interrupts
	static int EncoderMRPINB;  // no interrupt required	
	static int INTERRUPTID;
	// variables modified by interrupt handler must be declared as volatile
	static volatile int EncoderMRPosition;
	static volatile long interruptsReceived;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	static volatile short currentDirection; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	static long previousPosition;
	static void setupEncoderMR();	
	static void onEncoderMRInterrupt();
};

#endif