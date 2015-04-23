#ifndef MARS_EncoderFR
#define MARS_EncoderFR
#include "Arduino.h"
class EncoderFR
{
	// Class to handle EncoderFR data for the 2015 UP MARS devices.
	
	public:
	EncoderFR();
	static int getPosition();
	static int getRotationDirection();
	static int EncoderFRPINA;  // this pin needs to support interrupts
	static int EncoderFRPINB;  // no interrupt required	
	static int INTERRUPTID;
	// variables modified by interrupt handler must be declared as volatile
	static volatile int EncoderFRPosition;
	static volatile long interruptsReceived;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	static volatile short currentDirection; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	static long previousPosition;
	static void setupEncoderFR();	
	static void onEncoderFRInterrupt();
};

#endif