/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "EncoderFL.h"
//Constants from before:
#define CPR                  (400)    // EncoderFL cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant

// Static member definitions
int EncoderFL::EncoderFLPINA;
int EncoderFL::EncoderFLPINB;
int EncoderFL::INTERRUPTID;
volatile int EncoderFL::EncoderFLPosition;
volatile long EncoderFL::interruptsReceived;
volatile short EncoderFL::currentDirection; 
long EncoderFL::previousPosition;

EncoderFL::EncoderFL()
{
	EncoderFL::setupEncoderFL();
}

void EncoderFL::setupEncoderFL()
{
  EncoderFL::EncoderFLPINA = 3;
  EncoderFL::EncoderFLPINB = 4;
  EncoderFL::INTERRUPTID = 1;
  EncoderFL::EncoderFLPosition = 0;
  EncoderFL::interruptsReceived = 0;
  EncoderFL::currentDirection = -1;
  EncoderFL::previousPosition = 0;
  // inputs
  pinMode(EncoderFL::EncoderFLPINA, INPUT);
  pinMode(EncoderFL::EncoderFLPINB, INPUT);
  
  // interrupts
  attachInterrupt(EncoderFL::INTERRUPTID, &EncoderFL::onEncoderFLInterrupt, RISING);
}

int EncoderFL::getRotationDirection()
{
	return EncoderFL::currentDirection;
}

int EncoderFL::getPosition()
{
	return EncoderFL::EncoderFLPosition;
}

// interrupt function needs to do as little as possible
void EncoderFL::onEncoderFLInterrupt()
{
  // read both inputs
  int a = digitalRead(EncoderFL::EncoderFLPINA);
  int b = digitalRead(EncoderFL::EncoderFLPINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    --(EncoderFL::EncoderFLPosition);
    EncoderFL::currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    ++(EncoderFL::EncoderFLPosition);
    EncoderFL::currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  EncoderFL::EncoderFLPosition = EncoderFL::EncoderFLPosition % CPR;

  // track the number of interrupts
  ++(EncoderFL::interruptsReceived);
}
