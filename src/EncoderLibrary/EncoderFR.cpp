/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "EncoderFR.h"
//Constants from before:
#define CPR                  (400)    // EncoderFR cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant

// Static member definitions
int EncoderFR::EncoderFRPINA;
int EncoderFR::EncoderFRPINB;
int EncoderFR::INTERRUPTID;
volatile int EncoderFR::EncoderFRPosition;
volatile long EncoderFR::interruptsReceived;
volatile short EncoderFR::currentDirection; 
long EncoderFR::previousPosition;

EncoderFR::EncoderFR()
{
	EncoderFR::setupEncoderFR();
}

void EncoderFR::setupEncoderFR()
{
  EncoderFR::EncoderFRPINA = 20;
  EncoderFR::EncoderFRPINB = 5;
  EncoderFR::INTERRUPTID = 3;
  EncoderFR::EncoderFRPosition = 0;
  EncoderFR::interruptsReceived = 0;
  EncoderFR::currentDirection = -1;
  EncoderFR::previousPosition = 0;
  // inputs
  pinMode(EncoderFR::EncoderFRPINA, INPUT);
  pinMode(EncoderFR::EncoderFRPINB, INPUT);
  
  // interrupts
  attachInterrupt(EncoderFR::INTERRUPTID, &EncoderFR::onEncoderFRInterrupt, RISING);
}

int EncoderFR::getRotationDirection()
{
	return EncoderFR::currentDirection;
}

int EncoderFR::getPosition()
{
	return EncoderFR::EncoderFRPosition;
}

// interrupt function needs to do as little as possible
void EncoderFR::onEncoderFRInterrupt()
{
  // read both inputs
  int a = digitalRead(EncoderFR::EncoderFRPINA);
  int b = digitalRead(EncoderFR::EncoderFRPINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    --(EncoderFR::EncoderFRPosition);
    EncoderFR::currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    ++(EncoderFR::EncoderFRPosition);
    EncoderFR::currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  EncoderFR::EncoderFRPosition = EncoderFR::EncoderFRPosition % CPR;

  // track the number of interrupts
  ++(EncoderFR::interruptsReceived);
}
