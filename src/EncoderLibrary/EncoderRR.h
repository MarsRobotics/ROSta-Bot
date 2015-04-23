#ifndef MARS_EncoderRR
#define MARS_EncoderRR
#include "Arduino.h"
class EncoderRR
{
	// Class to handle EncoderRR data for the 2015 UP MARS devices.
	
	public:
	EncoderRR();
	static int getPosition();
	static int getRotationDirection();
	static int EncoderRRPINA;  // this pin needs to support interrupts
	static int EncoderRRPINB;  // no interrupt required	
	static int INTERRUPTID;
	// variables modified by interrupt handler must be declared as volatile
	static volatile int EncoderRRPosition;
	static volatile long interruptsReceived;
	// track direction: 1 = counter-clockwise; -1 = clockwise
	static volatile short currentDirection; // CLOCKWISE
	// track last position so we know whether it's worth printing new output
	static long previousPosition;
	static void setupEncoderRR();	
	static void onEncoderRRInterrupt();
};

#endif