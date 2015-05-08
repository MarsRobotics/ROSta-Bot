/* BucketWheelActuationDriver
Authors: Derek Schumacher, Fatima Dominguez, Jaimiey Sears
Begins by assuming the bucket wheel is at its lowest point (100%)
The user can specify a percentage to actuate to (0-100%)
The circuit uses HSI power screws and Microstep Driver- M6128 :
DIP Switch: [111001] (1 = down, 0 = up)
[current: 1.25; Microstep = 1 (Full-step)]
Send an 's' or 'S' to e-stop screw movement
The wiring scheme for the HSI power screws:
Red (A+)
Red/White (A-)
Green (B+)
Green/white (B-)
PUL+ (pin 9)
DIR+ (pin 8)
PUL- and DIR- grounded
ENA+ connected to ground
ENA- connected to pin 10 (+5V when turning, otherwise 0V)
*/

#include <ros.h>
#include <std_msgs/Int16.h>

#define ERROR_OUT_OF_RANGE -1

int DIRECTION_PIN = 8;
int PULSE_PIN = 9;
int ENABLE = 10;

//31914 steps translates to roughly (under) 100% top to bottom
#define FULL_STEPS 31914
//int stepCount = 0;
int desiredPercent = 0;

//double onePercent = FULL_STEPS/100.0;
//int desiredStep = 0;
int DELAY = 10;

//double currentPercent = 100;
int currentSteps = FULL_STEPS;

//variable which disables all motor movement when set to TRUE.
boolean e_stop = false;

// This node handle represents this arduino
ros::NodeHandle bucketWheelDriverNode;

void newManualCommandCallback(const std_msgs::Int16& newManualCommand)
{  
  int steps = newManualCommand.data;
  actuate(steps);
}
ros::Subscriber<std_msgs/Int16> commandSubscriber("BucketWheelCommand", &newManualCommandCallback);


void setup() {
  bucketWheelDriverNode.initNode();
  bucketWheelDriverNode.subscribe(commandSubscriber);
  
  
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  //  digitalWrite(ENABLE, HIGH); //the motor should not make loud screeching
  Serial.begin(9600);
  Serial.println("Send position to go to (percent):");
  printPosition();
}//setup

void loop() {

  if (Serial.available())
  {
    int readValue = Serial.parseInt();
    moveToPercent(readValue);

  }
  e_stop = false;
  
  //Sync with ROS
  bucketWheelDriverNode.spinOnce(); // Check for subscriber update/update timestamp

}//loop

//moves the wheel the down the number of steps specified
//negative steps will be interpred as moving the wheel up. 
void actuate(int steps) {
  //enable the controller
  digitalWrite(ENABLE, LOW);
  
  int dir = 0;
  //select which direction to go
  if (steps > 0) {
    digitalWrite(DIRECTION_PIN, LOW);
    dir = 1;
  }
  else {
    digitalWrite(DIRECTION_PIN, HIGH);
    dir = -1;
  }
  
  for(int i = 0; i < abs(steps); i++) {
    //leave if we say so
    interrupt();
    if (e_stop) break;
    
    //send pulse to controller
    delayMicroseconds(DELAY);
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(DELAY);
    digitalWrite(PULSE_PIN, LOW);

    //update and print position as we move
    currentSteps += dir;
    if (i%500 == 0) printPosition();
    
    //wait for 100 ms
    delayMicroseconds(1000000);
  }
  //report final position
  printPosition();

  digitalWrite(ENABLE, HIGH);
}

//used to read any commands that have been recieved
void interrupt() {
  if (Serial.available()) {
    char command = Serial.read();
    switch (command) {
      case 's':
        e_stop = true;
        break;
      case 'S':
        e_stop = true;
        break;
    }
  }
}

//short subroutine to print the current actuation position of the bucketwheel
void printPosition() {
  Serial.print("Current Position:\t");
  Serial.print(toPercent(currentSteps), DEC);
  Serial.println("%");
}


//moves to the given percent from the current position
int moveToPercent(int percent){
  //error check
  if (percent > 100 || percent < 0) return ERROR_OUT_OF_RANGE;
  
  //calculate number of steps and move.
  int stepsToMove = int(-(toPercent(currentSteps) - percent)/100.0 * FULL_STEPS);
  Serial.print("Moving to ");
  Serial.print(percent, DEC);
  Serial.print("% from ");
  Serial.print(toPercent(currentSteps), DEC);
  Serial.println("%");
  actuate(stepsToMove);
}

//takes a position in steps and converts it to a percentage
double toPercent(int steps) {
  return steps*100.0/FULL_STEPS;
}
