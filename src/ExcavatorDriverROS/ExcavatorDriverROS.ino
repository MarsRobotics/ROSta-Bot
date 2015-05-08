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

//#include <ros.h>
#include "ExcavatorDriverROS.h"


//<---------------- Bucket Wheel Actuation --------------->
const int ERROR_OUT_OF_RANGE = - 1;
const int FULL_STEPS = 31914;

const int BUCKET_SABERTOOTH_ADDRESS = 1;

int DIRECTION_PIN = 8;
int PULSE_PIN = 9;
int ENABLE = 10;

// 31914 steps translates to roughly (under) 100% top to bottom
int desiredPercent = 0;

int DELAY = 10;
int currentSteps = FULL_STEPS / 2; //TODO

//variable which disables all motor movement when set to TRUE.
boolean e_stop = false;

//<----------------- Excavator State machine ------------->
const int STOPPED = 0;
const int READY_TO_DRIVE = 1;
const int DRIVING = 2;
const int READY_TO_DIG = 3;
const int DIGGING =  4;
const int READY_TO_CONVEY = 5;
const int CONVEYING = 6;

const int DRIVE_SPEED = 20;
const char ROTATION_SPEED = 20;
const char BUCKET_SPEED = 20;
const char CONVEYOR_SPEED = 60;

int currentStatus = STOPPED;
unsigned long driveUntilTime = 0;
int conveyorRotationTime = 0;
long conveyorStopTime = 0;
int currentConveyorStatus = STOPPED;

//<---------------------- ROS variables ------------------->
/*
//
// ROS Node Initialization
//
ros::NodeHandle excavatorNode;

//
// ROS Publishers
//

//Sample status publisher
//command2ros::ExcavatorCommand excavatorTarget;
//ros::Publisher pubexcavatorStatus("???", &excavatorTarget);

//Publisher to print debug statements
std_msgs::String debugMsg;
ros::Publisher pubDebug("excavator_debugger", &debugMsg);

// Print error message to "excavator_debugger" topic
void print(char* errorMsg){
  debugMsg.data = errorMsg;
  pubDebug.publish(&debugMsg);
}

//
// ROS Subsribers
//
ros::Subscriber<command2ros::ExcavatorCommand> commandSubscriber("ExcavatorCommand", &newExcavatorCommandCallback);
command2ros::ExcavatorCommand lastExcavatorCommand;

void newExcavatorCommandCallback(const command2ros::ExcavatorCommand& newManualCommand)
{
 lastExcavatorCommand.fl_drive_speed = newManualCommand.fl_drive_speed;
 lastExcavatorCommand.fr_drive_speed = newManualCommand.fr_drive_speed;
 lastExcavatorCommand.bl_drive_speed = newManualCommand.bl_drive_speed;
 lastExcavatorCommand.br_drive_speed = newManualCommand.br_drive_speed;
 lastExcavatorCommand.drive_duration = newManualCommand.drive_duration;
 lastExcavatorCommand.e_stop = newManualCommand.e_stop;
 if(lastExcavatorCommand.e_stop)
 {
  stopAllMotors();
  currentStatus = STOPPED;
 }
 else
 {
   stopAllMotors();
   currentStatus = READY_TO_DRIVE;
 }
}

//Inbound wheel command
//command2ros::ExcavatorCommand wheelTarget;
*/

void stopAllMotors()
{
  for (int motorID = 0; motorID < NUM_EXCAVATOR_MOTORS; ++motorID) {
    driveForwards(motorID, 0);
  }
}

/**
 * Drives a given motor at a given speed in the "Forwards" direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveForwards(int motorID, int speed) {
  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  // Build the data packet:
  // Get the address and motor command ID from a predefined array.
  unsigned char address = MOTOR_ADDRESS[motorID];
  unsigned char command = MOTOR_COMMAND[motorID];

  // If the motor is connected backwards, we need to flip the command from 0/4 to 1/5:
  if (MOTOR_FLIPPED[motorID])
  {
    command += 1;
  }
  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;

  // Write the packet.
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(((char)speed));
  Serial3.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000);
}

/**
 * Drives a given motor at a given speed in the "backwards" direction.
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveBackwards(char motorID, char speed) {
  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  unsigned char address = MOTOR_ADDRESS[motorID];
  unsigned char command = MOTOR_COMMAND[motorID] + 1;

  // If the motor is connected backwards, we need to flip the command from 1/5 to 0/4:
  if (MOTOR_FLIPPED[motorID])
  {
    command -= 1;
  }
  unsigned char checksum = (address + command + speed) & 0b01111111;
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(speed);
  Serial3.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000);
}




// Function that takes a SIGNED speed value and converts it to an UNSIGNED drive command.
void helpDrive(char motorID, char speed)
{
  if (speed < 0)
  {
    driveBackwards(motorID, -speed);
  }
  else
  {
    driveForwards(motorID, speed);
  }
}

// Function to drive the wheels
/*
void driveTires()
{
 if(currentStatus == READY_TO_DRIVE)
 {
   helpDrive(FRONT_LEFT_DRIVE_MOTOR_ID, lastExcavatorCommand.fl_drive_speed);
   helpDrive(FRONT_RIGHT_DRIVE_MOTOR_ID, lastExcavatorCommand.fr_drive_speed);
   helpDrive(BACK_LEFT_DRIVE_MOTOR_ID, lastExcavatorCommand.bl_drive_speed);
   helpDrive(BACK_RIGHT_DRIVE_MOTOR_ID, lastExcavatorCommand.br_drive_speed);
   currentStatus = DRIVING;
 }
}
*/

//<---------------CODE TO SPIN BUCKET WHEEL---------------->
/**
 * Drives a given bucket-wheel motor at a given speed in a clockwise direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveClockwise(int motorID, int speed) {
  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  // Build the data packet:
  // Get the address and motor command ID from a predefined array.
  unsigned char address = ROTATE_MOTOR_ADDRESS[motorID];
  unsigned char command = ROTATE_MOTOR_COMMAND[motorID];

  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;

  // Write the packet.
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(((char)speed));
  Serial3.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000);
}

/**
 * Drives a given bucket-wheel motor at a given speed in a counterclockwise direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveCounterclockwise(char motorID, char speed) {

  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  unsigned char address = ROTATE_MOTOR_ADDRESS[motorID];
  unsigned char command = ROTATE_MOTOR_COMMAND[motorID] + 1;

  unsigned char checksum = (address + command + speed) & 0b01111111;
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(speed);
  Serial3.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000);
}


//<---------------CODE TO ACTUATE BUCKET WHEEL---------------->
//moves to the given percent from the current position
int moveToPercent(int percent) {
  //error check
  if (percent > 100 || percent < 0) return ERROR_OUT_OF_RANGE;

  //calculate number of steps and move.
  int stepsToMove = int(-(toPercent(currentSteps) - percent) / 100.0 * FULL_STEPS);

  actuate(stepsToMove);
}

//takes a position in steps and converts it to a percentage
double toPercent(int steps) {
  return steps * 100.0 / FULL_STEPS;
}

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

  for (int i = 0; i < abs(steps); i++) {
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

    //wait for 100 ms
    delayMicroseconds(1000000);
  }
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



//<------------------- DRIVE CONVEYOR ------------------------>
void driveConveyor(){
  if(conveyorRotationTime < 0){
    driveCounterclockwise(CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
  }
  else if(conveyorRotationTime > 0){
    driveClockwise(CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
  }
}

void stopConveyor(){
  driveCounterclockwise(CONVEYOR_MOTOR_ID, 0);
  currentConveyorStatus = STOPPED;
}



//<---------------------- UNIT TESTS ------------------------->
// Wheel unit test.
// Each wheel should rotate individually.
void unitTest()
{
  for (int i = 0; i < 4; ++i)
  {
    driveForwards(i, DRIVE_SPEED);
    delay(2000);
    driveBackwards(i, DRIVE_SPEED);
    delay(2000);
    driveForwards(i, 0);
    delay(600);
  }
}

// Drives all wheels forward, then all wheels back.
void unitTestDrive() {
  // drive all motors forward
  for (int i = 0; i < 4; ++i)
  {
    driveForwards(i, DRIVE_SPEED);
  }
  delay(4000);

  // drive all motors backwards
  for (int i = 0; i < 4; ++i)
  {
    driveBackwards(i, DRIVE_SPEED);
  }
  delay(4000);

  stopAllMotors();
}

// Spins the buketwheel clockwise, then counterclockwise
void unitTestBucketWheelSpin() {
  for (int i = 0; i < 4; ++i)
  {
    driveClockwise(i, BUCKET_SPEED);
  }
  delay(4000);

  for (int i = 0; i < 4; ++i)
  {
    driveCounterclockwise(i, BUCKET_SPEED);
  }

  delay(4000);

  for (int i = 0; i < 4; ++i)
  {
    driveClockwise(i, 0);
  }
  delay(600);
}

// Moves the bucket wheel up and down
void unitTestBucketWheelActuate() {
  moveToPercent(40);
  delay(4000);
  moveToPercent(50);
  delay(600);
}

// Moves the conveyor forwards and backwards
void unitTestConveyor(){
  driveClockwise(CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
  delaySeconds(3);
  driveClockwise(CONVEYOR_MOTOR_ID, 0);
  delaySeconds(2);
  driveCounterclockwise(CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
  delaySeconds(3);
  driveCounterclockwise(CONVEYOR_MOTOR_ID, 0);  
}

void delaySeconds(unsigned int n){
  unsigned int delayTime = 10000;
  long desiredMicroDelay = ((long)n) * 1000000L;
  long numCycles = desiredMicroDelay / (long)delayTime;
  for(long j = 0L; j < numCycles; j++){
    delayMicroseconds(delayTime);
  }
}

void setup()
{
  // Initialize ROS stuff
  /*
  excavatorNode.initNode();
  excavatorNode.subscribe(commandSubscriber);
  */

  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  
  // Initialize the serial connection to the saberteeth
  Serial3.begin(9600);
  //Serial.begin(9600);

  // Stop all the motors, if they were moving.
  // This must be done AFTER stopping all the motors.
  stopAllMotors();

  //<------ Run Unittests Here ------>
  unitTest();
  //unitTestDrive();
  //unitTestBucketWheelSpin();
  //unitTestBucketWheelActuate();
  unitTestConveyor();
  //stopAllMotors();
}

void loop()
{
  /*
  //Currently stopped, don't do anything.
  // ASSUMES the robot is stopped when currentStatus is set to STOPPED.
  switch (currentStatus)
  {
    case READY_TO_DRIVE:
      driveTires();
      driveUntilTime = millis() + (long)lastExcavatorCommand.drive_duration;
      break;
    case DRIVING:
      if (millis() > driveUntilTime)
      {
        stopAllMotors();
        // TODO: Change this to excavating, if necessary.
        currentStatus = STOPPED;
      }
      break;

    case STOPPED:
    //  // fall through
    default:
      // TODO: Convey / Dig
      break;
  }

  //TODO: Publish sensor / state data?
  //Sync with ROS
  excavatorNode.spinOnce(); // Check for subscriber update/update timestamp

  //Delay so we don't overload any serial buffers
  for (int i = 0; i < 7; i++) {
    delayMicroseconds(15000);
  }
  */
}


