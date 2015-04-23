/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "Encoder.h"
//Constants from before:
#define CPR                  (400)    // encoder cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant

// Static member definitions
int Encoder::ENCODERPINA;
int Encoder::ENCODERPINB;
int Encoder::INTERRUPTID;
volatile int Encoder::encoderPosition;
volatile long Encoder::interruptsReceived;
volatile short Encoder::currentDirection; 
long Encoder::previousPosition;

Encoder::Encoder()
{
	Encoder::setupEncoder();
}

void Encoder::setupEncoder()
{
  Encoder::ENCODERPINA = 3;
  Encoder::ENCODERPINB = 4;
  Encoder::INTERRUPTID = 1;
  Encoder::encoderPosition = 0;
  Encoder::interruptsReceived = 0;
  Encoder::currentDirection = -1;
  Encoder::previousPosition = 0;
  // inputs
  pinMode(Encoder::ENCODERPINA, INPUT);
  pinMode(Encoder::ENCODERPINB, INPUT);
  
  // interrupts
  attachInterrupt(Encoder::INTERRUPTID, &Encoder::onEncoderInterrupt, RISING);
}

int Encoder::getRotationDirection()
{
	return Encoder::currentDirection;
}

int Encoder::getPosition()
{
	return Encoder::encoderPosition;
}

// interrupt function needs to do as little as possible
void Encoder::onEncoderInterrupt()
{
  // read both inputs
  int a = digitalRead(Encoder::ENCODERPINA);
  int b = digitalRead(Encoder::ENCODERPINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    --(Encoder::encoderPosition);
    Encoder::currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    ++(Encoder::encoderPosition);
    Encoder::currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  Encoder::encoderPosition = Encoder::encoderPosition % CPR;

  // track the number of interrupts
  ++(Encoder::interruptsReceived);
}