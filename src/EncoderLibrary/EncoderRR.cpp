/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "EncoderRR.h"
//Constants from before:
#define CPR                  (400)    // EncoderRR cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant

// Static member definitions
int EncoderRR::EncoderRRPINA;
int EncoderRR::EncoderRRPINB;
int EncoderRR::INTERRUPTID;
volatile int EncoderRR::EncoderRRPosition;
volatile long EncoderRR::interruptsReceived;
volatile short EncoderRR::currentDirection; 
long EncoderRR::previousPosition;

EncoderRR::EncoderRR()
{
	EncoderRR::setupEncoderRR();
}

void EncoderRR::setupEncoderRR()
{
  EncoderRR::EncoderRRPINA = 21;
  EncoderRR::EncoderRRPINB = 4;
  EncoderRR::INTERRUPTID = 2;
  EncoderRR::EncoderRRPosition = 0;
  EncoderRR::interruptsReceived = 0;
  EncoderRR::currentDirection = -1;
  EncoderRR::previousPosition = 0;
  // inputs
  pinMode(EncoderRR::EncoderRRPINA, INPUT);
  pinMode(EncoderRR::EncoderRRPINB, INPUT);
  
  // interrupts
  attachInterrupt(EncoderRR::INTERRUPTID, &EncoderRR::onEncoderRRInterrupt, RISING);
}

int EncoderRR::getRotationDirection()
{
	return EncoderRR::currentDirection;
}

int EncoderRR::getPosition()
{
	return EncoderRR::EncoderRRPosition;
}

// interrupt function needs to do as little as possible
void EncoderRR::onEncoderRRInterrupt()
{
  // read both inputs
  int a = digitalRead(EncoderRR::EncoderRRPINA);
  int b = digitalRead(EncoderRR::EncoderRRPINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    --(EncoderRR::EncoderRRPosition);
    EncoderRR::currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    ++(EncoderRR::EncoderRRPosition);
    EncoderRR::currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  EncoderRR::EncoderRRPosition = EncoderRR::EncoderRRPosition % CPR;

  // track the number of interrupts
  ++(EncoderRR::interruptsReceived);
}
