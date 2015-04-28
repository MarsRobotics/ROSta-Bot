#include <ros.h>
// Include a custom command message.
#include <command2ros/ExcavatorCommand.h>


// Device constants
const int NUM_TRANSPORT_MOTORS = 4;
const unsigned char FRONT_LEFT_DRIVE_MOTOR_ID = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
const unsigned char BACK_LEFT_DRIVE_MOTOR_ID = 2;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_ID = 3;

const unsigned char FRONT_LEFT_DRIVE_MOTOR_ADDRESS = 131;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ADDRESS = 128;
const unsigned char BACK_LEFT_DRIVE_MOTOR_ADDRESS = 132;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_ADDRESS = 129;
const unsigned char MOTOR_ADDRESS[4] = {
  FRONT_LEFT_DRIVE_MOTOR_ADDRESS,
  FRONT_RIGHT_DRIVE_MOTOR_ADDRESS,
  BACK_LEFT_DRIVE_MOTOR_ADDRESS,
  BACK_RIGHT_DRIVE_MOTOR_ADDRESS};

const unsigned char FRONT_LEFT_DRIVE_MOTOR_COMMAND = 4;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_COMMAND = 4;
const unsigned char BACK_LEFT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_COMMAND = 4;
const unsigned char MOTOR_COMMAND[4] = {
  FRONT_LEFT_DRIVE_MOTOR_COMMAND,
  FRONT_RIGHT_DRIVE_MOTOR_COMMAND,
  BACK_LEFT_DRIVE_MOTOR_COMMAND,
  BACK_RIGHT_DRIVE_MOTOR_COMMAND};

const unsigned char FRONT_LEFT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_FLIPPED = 1;
const unsigned char BACK_LEFT_DRIVE_MOTOR_FLIPPED = 1;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char MOTOR_FLIPPED[4] = {
  FRONT_LEFT_DRIVE_MOTOR_FLIPPED,
  FRONT_RIGHT_DRIVE_MOTOR_FLIPPED,
  BACK_LEFT_DRIVE_MOTOR_FLIPPED,
  BACK_RIGHT_DRIVE_MOTOR_FLIPPED};


// State machine states
const int STOPPED = 0;
const int DRIVING = 1;
const int DIGGING = 2;
const int CONVEYING = 3;

int currentStatus = STOPPED;

const int DRIVE_SPEED = 20;
const int ROTATION_SPEED = 20;


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
ros::Subscriber<command2ros::ManualCommand> commandSubscriber("ExcavatorCommand", &newExcavatorCommandCallback);

void newExcavatorCommandCallback(const command2ros::ExcavatorCommand& newManualCommand)
{
 // TODO: excavator callback 
}


//Inbound wheel command
command2ros::ManualCommand wheelTarget;

void stopAllMotors()
{
    for(int motorID = 0; motorID < NUM_TRANSPORT_MOTORS; ++motorID) {
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
void driveForwards(int motorID, int speed){  
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
 * Drives a given motor at a given speed in the "backwards" direction.
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveBackwards(char motorID, char speed){ 
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

// Wheel unit test.
// Each wheel should rotate individually.
void unitTest()
{
  
  for(int i = 0; i < 4; ++i)
  {
    driveForwards(i,DRIVE_SPEED);
    delay(2000);
    driveBackwards(i,DRIVE_SPEED);
    delay(2000);
    driveForwards(i,0);
    delay(600);
  }
}

void setup()
{
  // Initialize the serial connection to the saberteeth
  Serial3.begin(9600);
  // Stop all the motors, if they were moving.
  // This must be done AFTER stopping all the motors.
  stopAllMotors(); 
  unitTest();
}

void loop()
{
 //Currently stopped, don't do anything.
  // ASSUMES the robot is stopped when currentStatus is set to STOPPED.
  if (currentStatus == STOPPED) {
    // do nothing
  }
  else if (currentStatus == DRIVING)
  {
   // TODO: Drive 
  }
  else if(currentStatus == DIGGING)
  {
    // TODO: Dig 
  }
  else if(currentStatus == CONVEYING)
  {
    // TODO: Convey 
  }
  //TODO: Publish sensor / state data?
  //Sync with ROS
  excavatorNode.spinOnce(); // Check for subscriber update/update timestamp

  //Delay so we don't overload any serial buffers
  for(int i = 0; i < 7; i++){
    delayMicroseconds(15000);
  } 
}


