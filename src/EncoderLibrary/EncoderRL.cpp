/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "EncoderRL.h"
//Constants from before:
#define CPR                  (400)    // EncoderRL cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant

// Static member definitions
int EncoderRL::EncoderRLPINA;
int EncoderRL::EncoderRLPINB;
int EncoderRL::INTERRUPTID;
volatile int EncoderRL::EncoderRLPosition;
volatile long EncoderRL::interruptsReceived;
volatile short EncoderRL::currentDirection; 
long EncoderRL::previousPosition;

EncoderRL::EncoderRL()
{
	EncoderRL::setupEncoderRL();
}

void EncoderRL::setupEncoderRL()
{
  EncoderRL::EncoderRLPINA = 3;
  EncoderRL::EncoderRLPINB = 8;
  EncoderRL::INTERRUPTID = 1;
  EncoderRL::EncoderRLPosition = 0;
  EncoderRL::interruptsReceived = 0;
  EncoderRL::currentDirection = -1;
  EncoderRL::previousPosition = 0;
  // inputs
  pinMode(EncoderRL::EncoderRLPINA, INPUT);
  pinMode(EncoderRL::EncoderRLPINB, INPUT);
  
  // interrupts
  attachInterrupt(EncoderRL::INTERRUPTID, &EncoderRL::onEncoderRLInterrupt, RISING);
}

int EncoderRL::getRotationDirection()
{
	return EncoderRL::currentDirection;
}

int EncoderRL::getPosition()
{
	return EncoderRL::EncoderRLPosition;
}

// interrupt function needs to do as little as possible
void EncoderRL::onEncoderRLInterrupt()
{
  // read both inputs
  int a = digitalRead(EncoderRL::EncoderRLPINA);
  int b = digitalRead(EncoderRL::EncoderRLPINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    ++(EncoderRL::EncoderRLPosition);
    EncoderRL::currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    --(EncoderRL::EncoderRLPosition);
    EncoderRL::currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  EncoderRL::EncoderRLPosition = EncoderRL::EncoderRLPosition % CPR;

  // track the number of interrupts
  ++(EncoderRL::interruptsReceived);
}
