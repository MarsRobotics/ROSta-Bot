/****************************************************************************************

Author:    Brenda A Bell
Modified by Derek Schumacher on March 24, 2015

****************************************************************************************/
#include "Encoder.h"
//Constants from before:
#define CPR                  (400)    // encoder cycles per revolution
#define CLOCKWISE            (-1)       // direction constant
#define COUNTER_CLOCKWISE    (1)       // direction constant
Encoder::Encoder(int ENCODER0PINA_in, int  ENCODER0PINB_in, int interruptID)
{
	this->ENCODER0PINA = ENCODER0PINA_in;
	this->ENCODER0PINB = ENCODER0PINB_in;
	setupEncoder(interruptID);
}

void Encoder::setupEncoder(int interruptID)
{

  // inputs
  pinMode(this->ENCODER0PINA, INPUT);
  pinMode(this->ENCODER0PINB, INPUT);
  
  // interrupts
  attachInterrupt(interruptID, <static_cast>(void*())&onInterrupt, RISING);
}

long Encoder::getPosition()
{
	return this->encoder0Position;
}

int Encoder::getRotationDirection()
{
	return this->currentDirection;
}

// interrupt function needs to do as little as possible
void onInterrupt()
{
  // read both inputs
  int a = digitalRead(this->ENCODER0PINA);
  int b = digitalRead(this->ENCODER0PINB);
    
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    --(this->encoder0Position);
    this->currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    ++(this->encoder0Position);
    this->currentDirection = CLOCKWISE;
  }

  // track 0 to 400
  this->encoder0Position = this->encoder0Position % CPR;

  // track the number of interrupts
  ++(this->interruptsReceived);
}
