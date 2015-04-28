/* 
 / This is a simple Arduino program that demonstrates the
 / "Packetized Serial" method of using the Sabertooth Motor
 / Controller. 
 /
 / SETUP:
 /  If ON is "up", then DIPs 1-3 should be DOWN. DIPs 4-6 are address DIPs,
 / and will vary by ther sabertooth.
 / Sabertooth[s] should be connected on S1 to TX3 on an Arduino Mega 2560.
 / This allows it to read 9600-baud commands from the Arduino Mega.
 / Be careful to use TX3: TX0 should be reserved for Arduino-Computer
 / communications.
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <command2ros/ManualCommand.h>
#include <EncoderFL.h>
#include <EncoderML.h>
#include <EncoderFR.h>
#include <EncoderMR.h>
#include <EncoderRL.h>
#include <EncoderRR.h>
#include "SabertoothDriverROS.h"


//The encoders we will use to monitor the angle of each articulation joint
EncoderFL efl;
EncoderML eml;
EncoderFR efr;
EncoderMR emr;
EncoderRL erl;
EncoderRR err;


//Control logic

//All proccesses killed, don't ever leave this state.
const int E_STOPPED = -1;

//Not driving any motors
const int STOPPED = 0;

//Articulating the wheels
const int ARTICULATING = 1;

//Done articulating, drive 
const int START_DRIVING = 2;

//Started articulating, drive
const int IS_DRIVING = 3;

//Received command to start rotation
const int START_ROTATING_CONVEYOR = 4;

//Started rotating conveyor
const int ROTATING_CONVEYOR = 5;

//Received command to rotate winch
const int START_ROTATING_WINCH = 6;

//Started rotating winch
const int ROTATING_WINCH = 7;

//The state the robot is currently in
int currentStatus = STOPPED;
long driveStopTime = 0;

//
// Articualtion Constants
//

//We want to get articulation join within this range (degrees) from the target
const int DELTA_START_RANGE = 8;
const int DELTA_STOP_RANGE = 3;

//When we are passing around the direction in which to articulate the wheels, we pass this value
const int ARTICULATION_DIRECTION_CLOCKWISE = -1;
const int ARTICULATION_DIRECTION_NONE = 0;
const int ARTICULATION_DIRECTION_COUNTER_CLOCKWISE = 1;

const int ENCODER_POSITIONS = 400;

//The speed 
const double ARTICULATION_DRIVE_SPEED = 6260.0 / 7521.0;
// This corresponds to the "maximum speed" available to the robot.
const int MAX_DRIVE_SPEED = 30;
bool motorInMotion[12];

//Conveyor and winch system constants
const char CONVEYOR_SPEED = 20;
const char WINCH_SPEED = 30;

int conveyorRotationTime = 0;
int winchRotationTime = 0;

long conveyorStopTime = 0;
long winchStopTime = 0;

//
// ROS initialization
//

// This node handle represents this arduino. 
ros::NodeHandle sabertoothDriverNode;

//
// ROS Publishers
// 

//Publisher to present the rotation of each of the articulation joints
command2ros::ManualCommand wheelStatus;
ros::Publisher pubwheelStatus("current_rotation", &wheelStatus);// current rotation data for each wheel. 
// This will publish when the angle is updated.

//Publisher to print debug statements
std_msgs::String debugMsg;
ros::Publisher pubDebug("sabertooth_debugger", &debugMsg);

//
// ROS Subsribers
//

//Inbound wheel command
command2ros::ManualCommand wheelTarget;

// Subscribers for intended drive velocity and direction
void newManualCommandCallback(const command2ros::ManualCommand& newManualCommand)
{      
  //TODO: Do we need to stop the robot at this point, Sherry and Matt H say no
  print("start cb");
    
  //Set articulation target values
  wheelTarget.fl_articulation_angle = newManualCommand.fl_articulation_angle;
  wheelTarget.ml_articulation_angle = newManualCommand.ml_articulation_angle;
  wheelTarget.rl_articulation_angle = newManualCommand.rl_articulation_angle;

  wheelTarget.fr_articulation_angle = newManualCommand.fr_articulation_angle;
  wheelTarget.mr_articulation_angle = newManualCommand.mr_articulation_angle;
  wheelTarget.rr_articulation_angle = newManualCommand.rr_articulation_angle;

  //Set wheel speeds
  wheelTarget.fl_drive_speed = newManualCommand.fl_drive_speed;
  wheelTarget.ml_drive_speed = newManualCommand.ml_drive_speed;
  wheelTarget.rl_drive_speed = newManualCommand.rl_drive_speed;

  wheelTarget.fr_drive_speed = newManualCommand.fr_drive_speed;
  wheelTarget.mr_drive_speed = newManualCommand.mr_drive_speed;
  wheelTarget.rr_drive_speed = newManualCommand.rr_drive_speed;

  //How long we will be driving from (in seconds)
  wheelTarget.drive_duration = newManualCommand.drive_duration;

  //Set E-Stop Command
  wheelTarget.e_stop = newManualCommand.e_stop;

  //Set the state of 
  if (wheelTarget.e_stop == true) {
    //TODO: Make this E_STOPPED for final product
    currentStatus = STOPPED;
    stopAllMotors(false);
  } 
  else if (needsToArticulate() == true) {
    currentStatus = ARTICULATING;
  } 
  else {
    currentStatus = START_DRIVING;
  }

}

void newConveyorCommandCallback(const std_msgs::Int16& newConveyorCommand){
  if(newConveyorCommand.data != 0){
    currentStatus = START_ROTATING_CONVEYOR;
    conveyorRotationTime = newConveyorCommand.data;
  }
}

void newWinchCommandCallback(const std_msgs::Int16& newWinchCommand){
  if(newWinchCommand.data != 0){
    currentStatus = START_ROTATING_WINCH;
    winchRotationTime = newWinchCommand.data;
  }
}


ros::Subscriber<command2ros::ManualCommand> commandSubscriber("ManualCommand", &newManualCommandCallback);

ros::Subscriber<std_msgs::Int16> conveyorSubscriber("ConveyorCommand", &newConveyorCommandCallback);

ros::Subscriber<std_msgs::Int16> winchSubscriber("ConveyorCommand", &newWinchCommandCallback);


// Print error message to "sabertooth_debugger" topic
void print(char* errorMsg){
  debugMsg.data = errorMsg;
  pubDebug.publish(&debugMsg);
}

void setupWheelStatus() {
  wheelStatus.fl_articulation_angle = 0;
  wheelStatus.fr_articulation_angle = 180;
  wheelStatus.ml_articulation_angle = 0;
  wheelStatus.mr_articulation_angle = 180;
  wheelStatus.rl_articulation_angle = 0;
  wheelStatus.rr_articulation_angle = 180;

  wheelStatus.fl_drive_speed = 0;
  wheelStatus.fr_drive_speed = 0;
  wheelStatus.ml_drive_speed = 0;
  wheelStatus.mr_drive_speed = 0;
  wheelStatus.rl_drive_speed = 0;
  wheelStatus.rr_drive_speed = 0;
}

/**
 * Checks the current angle of each of the articulation joints against their target value
 *    if the difference betwen any of them exceed the range given by DELTA_RANGE then we need 
 *    to articulate "return true"
 */
bool needsToArticulate() {

  int delta;
  int moveRange;
  //Check if the difference between target and actual articulation exceeds our given range
  delta = wheelStatus.fl_articulation_angle - wheelTarget.fl_articulation_angle;  //FL
  moveRange = (motorInMotion[FRONT_LEFT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.ml_articulation_angle - wheelTarget.ml_articulation_angle;  //ML
  moveRange = (motorInMotion[MIDDLE_LEFT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.rl_articulation_angle - wheelTarget.rl_articulation_angle;  //RL
  moveRange = (motorInMotion[REAR_LEFT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.fr_articulation_angle - wheelTarget.fr_articulation_angle;  //FR
  moveRange = (motorInMotion[FRONT_RIGHT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.mr_articulation_angle - wheelTarget.mr_articulation_angle;  //MR
  moveRange = (motorInMotion[MIDDLE_RIGHT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }
  
  delta = wheelStatus.rr_articulation_angle - wheelTarget.rr_articulation_angle;  //RR
  moveRange = (motorInMotion[REAR_RIGHT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  return false;
}

void articulateAllWheels() {
  int delta;
  int direction;

  direction = getArticulationDirection(FRONT_LEFT_DRIVE_MOTOR_ID, wheelStatus.fl_articulation_angle, wheelTarget.fl_articulation_angle); //FL
  articulateWheel(FRONT_LEFT_DRIVE_MOTOR_ID, direction);

  direction = getArticulationDirection(MIDDLE_LEFT_DRIVE_MOTOR_ID, wheelStatus.ml_articulation_angle, wheelTarget.ml_articulation_angle); //ML
  articulateWheel(MIDDLE_LEFT_DRIVE_MOTOR_ID, direction);

  direction = getArticulationDirection(REAR_LEFT_DRIVE_MOTOR_ID, wheelStatus.rl_articulation_angle, wheelTarget.rl_articulation_angle); //RL
  articulateWheel(REAR_LEFT_DRIVE_MOTOR_ID, direction);

  direction = getArticulationDirection(FRONT_RIGHT_DRIVE_MOTOR_ID, wheelStatus.fr_articulation_angle, wheelTarget.fr_articulation_angle); //FR
  articulateWheel(FRONT_RIGHT_DRIVE_MOTOR_ID, direction);

  direction = getArticulationDirection(MIDDLE_RIGHT_DRIVE_MOTOR_ID, wheelStatus.mr_articulation_angle, wheelTarget.mr_articulation_angle); //MR
  articulateWheel(MIDDLE_RIGHT_DRIVE_MOTOR_ID, direction);

  direction = getArticulationDirection(REAR_RIGHT_DRIVE_MOTOR_ID, wheelStatus.rr_articulation_angle, wheelTarget.rr_articulation_angle); //RR
  articulateWheel(REAR_RIGHT_DRIVE_MOTOR_ID, direction);
}

/**
 *
 */
int getArticulationDirection(int motorID, int from, int to) {

  int delta = from-to;
  
  int moveRange = (motorInMotion[motorID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta >= -moveRange && delta <= moveRange) {
    return ARTICULATION_DIRECTION_NONE;
  }

  // Calculations are made by "normalizing" the data so that we are always starting from "0" 
  // and heading towards the target

  // If the current articulation angle is between 0 and 180, then we want to shift the 
  // whole frame of reference clockwise

  //Shift "to" and "from" so that "to" is the angle that we need to turn and "from" is 0
  if(from >= 0 && from <= 180){

    // shift "to" by however much "from" shifted (counter clockwise)
    to -= from;
    if(to < 0){
      to += 360;
    }

    // shift "from" to 0 degrees
    from = 0;
  }
  else{
    delta = 360 - from;

    // shift "from" to 0 degrees
    from = 0;

    // shift "to" by however much "from" shifted (clockwise)
    to += delta;
    if(to > 360){
      to -= 360;
    }
  }

  //Now we compare the two distances and move in the diection that is fastest
  int counterClockwiseDistance = to;
  int clockwiseDistance = 360 - to;

  if(counterClockwiseDistance > clockwiseDistance){
    return ARTICULATION_DIRECTION_CLOCKWISE;
  }
  else{
    return ARTICULATION_DIRECTION_COUNTER_CLOCKWISE;
  }
}

/**
 *
 */
void articulateWheel(int motorID, int direction) {

  int articulationSpeed = 30*ARTICULATION_DRIVE_SPEED;
  int wheelSpeed = 30;

  //TODO: Make a constant
  int articulationID = motorID + 6;

  if (direction == ARTICULATION_DIRECTION_CLOCKWISE) {
    driveClockwise(articulationID, articulationSpeed);
    driveClockwise(motorID, wheelSpeed);
  } 
  else if (direction == ARTICULATION_DIRECTION_COUNTER_CLOCKWISE) {
    driveCounterclockwise(articulationID, articulationSpeed);
    driveCounterclockwise(motorID, wheelSpeed);
  }
  else if(direction == ARTICULATION_DIRECTION_NONE){
    driveCounterclockwise(articulationID, 0);
    driveCounterclockwise(motorID, 0);
  }
}

//TODO: Rename to driveAllWheels
void driveAllMotors() {

  //Set wheel speeds
  driveMotor(FRONT_LEFT_DRIVE_MOTOR_ID, wheelTarget.fl_drive_speed*-1);
  driveMotor(MIDDLE_LEFT_DRIVE_MOTOR_ID, wheelTarget.ml_drive_speed*-1);
  driveMotor(REAR_LEFT_DRIVE_MOTOR_ID, wheelTarget.rl_drive_speed*-1);

  driveMotor(FRONT_RIGHT_DRIVE_MOTOR_ID, wheelTarget.fr_drive_speed);
  driveMotor(MIDDLE_RIGHT_DRIVE_MOTOR_ID, wheelTarget.mr_drive_speed);
  driveMotor(REAR_RIGHT_DRIVE_MOTOR_ID, wheelTarget.rr_drive_speed);

}

//TODO: Rename to driveWheel
void driveMotor(int motorID, int speed) {

  if (speed < 0) {
    speed = speed * -1;
    driveClockwise(motorID, speed);
  } 
  else {
    speed = speed;
    driveCounterclockwise(motorID, speed);
  }

}

/**
 * Stops all the motors (Articulation and Wheels) and chages the current state to reflect the stop
 *
 * Parameters:
 *  EStop - Weather or not this is an E-Stop or a normal end of command stop
 */
void stopAllMotors(bool EStop) {

  if (EStop == true) {
    currentStatus = E_STOPPED;
  } 
  else {
    currentStatus = STOPPED;
  }

  //Stop all motors 
  const int speed = 0;
  for(int motorID = 0; motorID <= 11; ++motorID) {
    driveClockwise(motorID, speed);
  }

}

void driveConveyor(){
  if(conveyorRotationTime < 0){
    driveCounterclockwise(LEFT_CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
    driveCounterclockwise(RIGHT_CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
  }
  else if(conveyorRotationTime > 0){
    driveClockwise(LEFT_CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
    driveClockwise(RIGHT_CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
  }
}

void stopConveyor(){
  driveCounterclockwise(LEFT_CONVEYOR_MOTOR_ID, 0);
  driveCounterclockwise(RIGHT_CONVEYOR_MOTOR_ID, 0);
}

void driveWinch(){
   if(winchRotationTime < 0){
    driveClockwise(WINCH_MOTOR_ID, WINCH_SPEED);
  }
  else if(conveyorRotationTime > 0){
    driveClockwise(WINCH_MOTOR_ID, WINCH_SPEED);
  }
}

void stopWinch(){
  driveCounterclockwise(WINCH_MOTOR_ID, 0);
}


/**
 * Drives a given motor at a given speed in a clockwise direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveClockwise(int motorID, int speed){
  if(0 == speed)
  {
   motorInMotion[motorID] = false; 
  }
  else
  {
    motorInMotion[motorID] = true;
  }
  
  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  // Build the data packet:
  // Get the address and motor command ID from a predefined array.
  unsigned char address = MOTOR_ADDRESS[motorID];
  unsigned char command = MOTOR_COMMAND[motorID];
  // If the motor is connected backwards, we need to flip the command from 0/4 to 1/5:
  if(MOTOR_FLIPPED[motorID])
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
 * Drives a given motor at a given speed in a counterclockwise direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveCounterclockwise(char motorID, char speed){ 
  if(0 == speed)
  {
   motorInMotion[motorID] = false; 
  }
  else
  {
    motorInMotion[motorID] = true;
  }
  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  unsigned char address = MOTOR_ADDRESS[motorID];
  unsigned char command = MOTOR_COMMAND[motorID] + 1;
  // If the motor is connected backwards, we need to flip the command from 1/5 to 0/4:
  if(MOTOR_FLIPPED[motorID])
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

/**
* Gets the last known position of each articulation joint, and updates acordingly.
*/
void updateArticulationValues()
{
  
  /*
  // Code for command line testing purposes
  wheelStatus.ml_articulation_angle = wheelTarget.ml_articulation_angle;
  wheelStatus.rl_articulation_angle = wheelTarget.rl_articulation_angle;
  wheelStatus.fl_articulation_angle = wheelTarget.fl_articulation_angle;
  wheelStatus.mr_articulation_angle = (int)wheelTarget.mr_articulation_angle;
  wheelStatus.rr_articulation_angle = (int)wheelTarget.rr_articulation_angle;
  wheelStatus.fr_articulation_angle = (int)wheelTarget.fr_articulation_angle;
  */
  
  int encoderPostition = EncoderFL::getPosition();
  if (encoderPostition < 0)
  {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.fl_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderML::getPosition();
  if (encoderPostition < 0)
  {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.ml_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderRL::getPosition();
  if (encoderPostition < 0)
  {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.rl_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderFR::getPosition() - 200; // Subtract 200 b/c the right side is flipped.
  if (encoderPostition < 0)
  {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.fr_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderMR::getPosition() - 200;
  if (encoderPostition < 0)
  {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.mr_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderRR::getPosition() - 200;
  if (encoderPostition < 0)
  {
    encoderPostition += ENCODER_POSITIONS;
  }

  wheelStatus.rr_articulation_angle = (int)((encoderPostition * 9L) / 10L);
}

void unitTest(){
  // delay after set up
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }

  // loop through and test all wheels
  for(int i = 0; i < 6; i++){
    // clockwise
    driveClockwise(i, 20);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }

    // counterclockwise
    driveCounterclockwise(i, 20);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }   

    // stop
    driveCounterclockwise(i,0);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }
  }

  for(int i = 6; i < 12; ++i){
    // clockwise
    driveClockwise(i, 30);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }   

    // counterclockwise
    driveCounterclockwise(i, 30);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    } 

    // stop 
    driveClockwise(i,0);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }
  }
}

void unitTest2(){
  // delay after startup
  for(int j = 0; j < 300; j++){
    delayMicroseconds(15000);
  }


  digitalWrite(13, HIGH);    // turn the LED off by making the voltage LOW
  driveCounterclockwise(0, 20);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }

  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
  driveCounterclockwise(0,0);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  driveClockwise(0, 20);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }  

  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
  driveClockwise(0,0);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }
}


void delaySeconds(long n){
  int delayTime = 15000;
  long desiredMicroDelay = n * 100000;
  long numCycles = desiredMicroDelay / (long)delayTime;
  for(int j = 0; j < numCycles; j++){
    delayMicroseconds(delayTime);
  }
}



/**
 * Is called on starting the arduino.
 * 
 */
void setup(){
  // To start, no motor is moving.
  for(int i = 0; i < 12; ++i)
  {
   motorInMotion[i] = false; 
  }

  // Setup the encoders
  EncoderFL::setupEncoderFL();
  EncoderML::setupEncoderML();
  EncoderRL::setupEncoderRL();
  EncoderFR::setupEncoderFR();
  EncoderMR::setupEncoderMR();
  EncoderRR::setupEncoderRR();

  //Initialize the ROS Node
  sabertoothDriverNode.initNode();
  sabertoothDriverNode.subscribe(commandSubscriber);
  sabertoothDriverNode.advertise(pubwheelStatus);
  sabertoothDriverNode.advertise(pubDebug);


  // Open communication with Saberteeth
  Serial3.begin(9600);
  
  stopAllMotors(false);
  //unitTest();


  // Initialize the message
  setupWheelStatus();
}

// This program does whatever ROS directs it to do.
// So far, it drives the robot forwards and backwards.
void loop(){
  updateArticulationValues(); 
  
  //We are E-Stopped, don't respond to future commands.
  if (currentStatus == E_STOPPED) {
    //TODO: This will never happen right now, may want for competition though
    return;
  } 

  //Currently stopped, don't do anything.
  // ASSUMES the robot is stopped when currentStatus is set to STOPPED.
  if (currentStatus == STOPPED) {
    // do nothing
  }
  
  if(currentStatus == START_ROTATING_CONVEYOR){
    conveyorStopTime = millis() + (long)conveyorRotationTime;   
    driveConveyor();
    currentStatus = ROTATING_CONVEYOR;
  }
  
  if(currentStatus == ROTATING_CONVEYOR){
    if(millis() >= conveyorStopTime){
      stopConveyor();
    }
  }
  
  if(currentStatus == START_ROTATING_WINCH){
    conveyorStopTime = millis() + (long)winchRotationTime;   
    driveWinch();
    currentStatus = ROTATING_CONVEYOR;
  }
  
  if(currentStatus == ROTATING_WINCH){
    if(millis() >= winchStopTime){
      stopWinch();
    }
  }

  if (currentStatus == ARTICULATING) {
    print("needs to articulate: true");
    if (needsToArticulate() == true) {
      articulateAllWheels();
    } 
    else {
      print("needs to articulate: false");
      stopAllMotors(false);
      currentStatus = START_DRIVING;
    }
  }

  if (currentStatus == START_DRIVING) {
    //TODO: Check with MEs about possibility of losing correct articulation (by going over an obstacle or something)

    //Check if we need to articulate
    // if (needsToArticulate()) {
    //   //Set state to articulate and don't drive
    //   currentStatus = ARTICULATING;
	//   // TODO: Does this need a rosnode.spin() ?
    //   return;
    // }

    print("current status: start_driving");
    //Set stop drive time
    driveStopTime = millis() + (long)wheelTarget.drive_duration*1000;
    
    //Send drive start command
    driveAllMotors();

    currentStatus = IS_DRIVING;
  }

  if (currentStatus == IS_DRIVING) {
    if (millis() >= driveStopTime) {
      print("over time limit: stop driving");
      stopAllMotors(false);
    }
  }

  pubwheelStatus.publish(&wheelStatus);// current rotation data for each wheel. 

  //Sync with ROS
  sabertoothDriverNode.spinOnce(); // Check for subscriber update/update timestamp

  //Delay so we don't overload any serial buffers
  for(int i = 0; i < 7; i++){
    delayMicroseconds(15000);
  }
}




