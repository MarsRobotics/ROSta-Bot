/* RotatingCamera

Authors: Derek Schumacher, Matt Delaney, and Fatima Dominguez

Reads in a degree value from ROS and translates it to 
a number of steps that will rotate the motor to the desired 
degree.

The circuit using a NEMA 14 and Microstep Driver- M6128 :
[current: 1.25; Microstep = 1 (Full-step)]

Red (A+)
Green (A-)
Yellow (B+)
Blue (B-)

PUL+ (pin 8)
DIR+ (pin 9)

ENA+ connected to ground
ENA- connected to pin 10 (+5V when turning, otherwise 0V)

*/
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
int DIRECTION_PIN = 8;
int PULSE_PIN = 9;
int ENABLE = 10;
int stepCount = 0;
int desiredDegrees = 0;
int beaconDegreeOffset = 0;
double stepDegree = 0.094;
int desiredStep = 0;
// In testing it seems this is unecessary.
int DELAY = 0;

int CLOCKWISE = HIGH;
int COUNTERCLOCKWISE = LOW;

void desiredCameraPosChanged(const std_msgs::Int32& newPos);
void driveUp(void);
void driveDown(void);

ros::Subscriber<std_msgs::Int32> cvDegreeReader("camera_angle", &desiredCameraPosChanged);

void setup(){

  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  // HIGH on Enable = DISABLED. LOW on Enable = ENABLED.
  digitalWrite(ENABLE, HIGH); //the motor should not make loud screeching
  nh.initNode();
  nh.subscribe(cvDegreeReader);
}

void loop()
{
	nh.spinOnce();
	delayMicroseconds(1000);
}

void desiredCameraPosChanged( const std_msgs::Int32& newPos){
    desiredDegrees = newPos.data;
	// Step = Degrees / steps per degree * gear ratio
    desiredStep = int(desiredDegrees/stepDegree)*19;
    Serial.print("Going to degree: ");
    Serial.println(desiredDegrees); 
  if(stepCount == desiredStep)
  {
	  return;
  }
  else if (stepCount < desiredStep)
  {
	  driveUp();
  }
  else
  {
	  driveDown();
  }
}
  
  // increasing the count means we want to move clockwise (right)
  void driveUp()
  {
	digitalWrite(ENABLE,LOW);
	digitalWrite(DIRECTION_PIN, CLOCKWISE);
	while(stepCount < desiredStep)
	{
		delayMicroseconds(DELAY);
		digitalWrite(PULSE_PIN, HIGH);
		delayMicroseconds(DELAY);
		digitalWrite(PULSE_PIN, LOW);
		++stepCount;
	}
  }
  
  // Decreasing the step means we want to move counter-clockwise (left).
  void driveDown()
  {
	digitalWrite(ENABLE,LOW);
	digitalWrite(DIRECTION_PIN, COUNTERCLOCKWISE);
	while(stepCount > desiredStep)
	{
		delayMicroseconds(DELAY);
		digitalWrite(PULSE_PIN, HIGH);
		delayMicroseconds(DELAY);
		digitalWrite(PULSE_PIN, LOW);
		--stepCount;
	}
  }

 
