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
 
 PUL+ (pin 9)
 DIR+ (pin 8)
 
 ENA+ connected to 5V
 ENA- connected to pin 10 (+5V when turning, otherwise 0V)
 
 */
#include <ros.h>
#include <std_msgs/Int32.h>


const boolean USE_ROS = true;

ros::NodeHandle nh;
const int DIRECTION_PIN = 8;
const int PULSE_PIN = 9;
const int ENABLE = 10;
int stepCount = 0;
int desiredDegrees = 0;
int beaconDegreeOffset = 0;
double stepDegree = 0.094;
int desiredStep = 0;
// In testing it seems this is unecessary.
const int DELAY = 10;

const int MAX_STEP_VALUE = 72765;

const int CLOCKWISE = HIGH;
const int COUNTERCLOCKWISE = LOW;

void desiredCameraPosChanged(const std_msgs::Int32& newPos);
void driveUp(void);
void driveDown(void);

ros::Subscriber<std_msgs::Int32> cvDegreeReader("target_camera_angle", &desiredCameraPosChanged);

std_msgs::Int32 currentCameraAngle;
ros::Publisher cvDegreePublisher("current_camera_angle", &currentCameraAngle);

void setup(){

  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  // HIGH on Enable = DISABLED. LOW on Enable = ENABLED.
  digitalWrite(ENABLE, HIGH); //the motor should not make loud screeching
  if (USE_ROS) {
    nh.initNode();
    nh.subscribe(cvDegreeReader);
    nh.advertise(cvDegreePublisher);
  } else {
    Serial.begin(9600);
  }
}

void loop()
{
  if (USE_ROS) {
    nh.spinOnce();
  } else {
    if(Serial.available())
    {
      rotateToFaceAngle(Serial.parseInt());
    }
  }
  
  delayMicroseconds(1000);
}

void rotateToFaceAngle(int angle)
{
  while(angle < 0)
  {
    angle += 360;
  }
  desiredDegrees = angle % 360;
  // Step = Degrees / steps per degree * gear ratio
  desiredStep = int(desiredDegrees/stepDegree)*19;
  if(stepCount == desiredStep)
  {
    return;
  }
  else if ((abs(desiredStep - stepCount) < int(180/stepDegree)*19 ) || 
  ((desiredStep + MAX_STEP_VALUE - stepCount) < 180) )
  {
    driveUp();
  }
  else
  {
    driveDown();
  }
}

void desiredCameraPosChanged( const std_msgs::Int32& newPos){
  desiredDegrees = newPos.data;
  currentCameraAngle.data = desiredDegrees;
  rotateToFaceAngle(desiredDegrees);
  cvDegreePublisher.publish(&currentCameraAngle);
}



// increasing the count means we want to move clockwise (right)
void driveUp()
{
  digitalWrite(ENABLE,LOW);
  digitalWrite(DIRECTION_PIN, CLOCKWISE);
  while(stepCount != desiredStep)
  {
    delayMicroseconds(DELAY);
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(DELAY);
    digitalWrite(PULSE_PIN, LOW);
    stepCount = (stepCount + 1) % MAX_STEP_VALUE;
  }
}

// Decreasing the step means we want to move counter-clockwise (left).
void driveDown()
{
  digitalWrite(ENABLE,LOW);
  digitalWrite(DIRECTION_PIN, COUNTERCLOCKWISE);
  while(stepCount != desiredStep)
  {
    delayMicroseconds(DELAY);
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(DELAY);
    digitalWrite(PULSE_PIN, LOW);
    --stepCount;
    if(stepCount < 0)
    {
      stepCount += MAX_STEP_VALUE;
    }
  }
}



