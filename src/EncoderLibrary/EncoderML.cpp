/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "EncoderML.h"
//Constants from before:
#define CPR                  (400)    // EncoderML cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant

// Static member definitions
int EncoderML::EncoderMLPINA;
int EncoderML::EncoderMLPINB;
int EncoderML::INTERRUPTID;
volatile int EncoderML::EncoderMLPosition;
volatile long EncoderML::interruptsReceived;
volatile short EncoderML::currentDirection; 
long EncoderML::previousPosition;

EncoderML::EncoderML()
{
	EncoderML::setupEncoderML();
}

void EncoderML::setupEncoderML()
{
  EncoderML::EncoderMLPINA = 18;
  EncoderML::EncoderMLPINB = 7;
  EncoderML::INTERRUPTID = 5;
  EncoderML::EncoderMLPosition = 0;
  EncoderML::interruptsReceived = 0;
  EncoderML::currentDirection = -1;
  EncoderML::previousPosition = 0;
  // inputs
  pinMode(EncoderML::EncoderMLPINA, INPUT);
  pinMode(EncoderML::EncoderMLPINB, INPUT);
  
  // interrupts
  attachInterrupt(EncoderML::INTERRUPTID, &EncoderML::onEncoderMLInterrupt, RISING);
}

int EncoderML::getRotationDirection()
{
	return EncoderML::currentDirection;
}

int EncoderML::getPosition()
{
	return EncoderML::EncoderMLPosition;
}

// interrupt function needs to do as little as possible
void EncoderML::onEncoderMLInterrupt()
{
  // read both inputs
  int a = digitalRead(EncoderML::EncoderMLPINA);
  int b = digitalRead(EncoderML::EncoderMLPINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    ++(EncoderML::EncoderMLPosition);
    EncoderML::currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    --(EncoderML::EncoderMLPosition);
    EncoderML::currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  EncoderML::EncoderMLPosition = EncoderML::EncoderMLPosition % CPR;

  // track the number of interrupts
  ++(EncoderML::interruptsReceived);
}
