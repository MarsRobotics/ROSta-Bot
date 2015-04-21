/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "Encoder.h"


//Constants from before:
#define CPR                  400    // encoder cycles per revolution
#define CLOCKWISE            -1       // direction constant
#define COUNTER_CLOCKWISE    1       // direction constant

Encoder::Encoder(int ENCODER0PINA_in, int  ENCODER0PINB_in, int interruptID)
{
	ENCODER0PINA = ENCODER0PINA_in;
	ENCODER0PINB = ENCODER0PINB_in;
	setupEncoder();
}

void Encoder::setupEncoder()
{

  // inputs
  pinMode(ENCODER0PINA, INPUT);
  pinMode(ENCODER0PINB, INPUT);
  pinMode(ENCODER0INDEX, INPUT);
  
  // interrupts
  attachInterrupt(interruptID, Encoder::onInterrupt, RISING);
}

long Encoder::getPosition()
{
	return encoder0Position;
}

int Encoder::getRotationDirection()
{
	return currentDirection;
}

// interrupt function needs to do as little as possible
void Encoder::onInterrupt()
{
  // read both inputs
  int a = digitalRead(ENCODER0PINA);
  int b = digitalRead(ENCODER0PINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    --encoder0Position;
    currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    ++encoder0Position;
    currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  encoder0Position = encoder0Position % CPR;

  // track the number of interrupts
  ++interruptsReceived;
}
