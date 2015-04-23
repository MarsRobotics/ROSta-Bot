/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "EncoderMR.h"
//Constants from before:
#define CPR                  (400)    // EncoderMR cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant

// Static member definitions
int EncoderMR::EncoderMRPINA;
int EncoderMR::EncoderMRPINB;
int EncoderMR::INTERRUPTID;
volatile int EncoderMR::EncoderMRPosition;
volatile long EncoderMR::interruptsReceived;
volatile short EncoderMR::currentDirection; 
long EncoderMR::previousPosition;

EncoderMR::EncoderMR()
{
	EncoderMR::setupEncoderMR();
}

void EncoderMR::setupEncoderMR()
{
  EncoderMR::EncoderMRPINA = 3;
  EncoderMR::EncoderMRPINB = 4;
  EncoderMR::INTERRUPTID = 1;
  EncoderMR::EncoderMRPosition = 0;
  EncoderMR::interruptsReceived = 0;
  EncoderMR::currentDirection = -1;
  EncoderMR::previousPosition = 0;
  // inputs
  pinMode(EncoderMR::EncoderMRPINA, INPUT);
  pinMode(EncoderMR::EncoderMRPINB, INPUT);
  
  // interrupts
  attachInterrupt(EncoderMR::INTERRUPTID, &EncoderMR::onEncoderMRInterrupt, RISING);
}

int EncoderMR::getRotationDirection()
{
	return EncoderMR::currentDirection;
}

int EncoderMR::getPosition()
{
	return EncoderMR::EncoderMRPosition;
}

// interrupt function needs to do as little as possible
void EncoderMR::onEncoderMRInterrupt()
{
  // read both inputs
  int a = digitalRead(EncoderMR::EncoderMRPINA);
  int b = digitalRead(EncoderMR::EncoderMRPINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    --(EncoderMR::EncoderMRPosition);
    EncoderMR::currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    ++(EncoderMR::EncoderMRPosition);
    EncoderMR::currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  EncoderMR::EncoderMRPosition = EncoderMR::EncoderMRPosition % CPR;

  // track the number of interrupts
  ++(EncoderMR::interruptsReceived);
}
