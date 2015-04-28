#include <ros.h>
// TODO: Include a custom message command.

// Device constants
const int NUM_TRANSPORT_MOTORS = 4;
const unsigned char FRONT_LEFT_DRIVE_MOTOR_ID = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
const unsigned char BACK_LEFT_DRIVE_MOTOR_ID = 2;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_ID = 3;

const unsigned char FRONT_LEFT_DRIVE_MOTOR_ADDRESS = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ADDRESS = 0;
const unsigned char BACK_LEFT_DRIVE_MOTOR_ADDRESS = 0;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_ADDRESS = 0;
const unsigned char MOTOR_ADDRESS[4] = {
  FRONT_LEFT_MOTOR_ADDRESS,
  FRONT_RIGHT_MOTOR_ADDRESS,
  BACK_LEFT_MOTOR_ADDRESS,
  BACK_RIGHT_MOTOR_ADDRESS};

const unsigned char FRONT_LEFT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char BACK_LEFT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char MOTOR_COMMAND[4] = {
  FRONT_LEFT_MOTOR_COMMAND,
  FRONT_RIGHT_MOTOR_COMMAND,
  BACK_LEFT_MOTOR_COMMAND,
  BACK_RIGHT_MOTOR_COMMAND};

const unsigned char FRONT_LEFT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char BACK_LEFT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char MOTOR_FLIPPED[4] = {
  FRONT_LEFT_MOTOR_FLIPPED,
  FRONT_RIGHT_MOTOR_FLIPPED,
  BACK_LEFT_MOTOR_FLIPPED,
  BACK_RIGHT_MOTOR_FLIPPED};


// State machine states
const int STOPPED = 0;
const int ROTATING = 1;
const int DRIVING = 2;
const int DIGGING = 3;
const int CONVEYING = 4;

int currentStatus = STOPPED;

const int DRIVE_SPEED = 20;
const int ROTATION_SPEED = 20;

// TODO: ROS nodes and stuff: init, callback functions, etc

void stopAllMotors()
{
    for(int motorID = 0; motorID < NUM_TRANSPORT_MOTORS; ++motorID) {
    driveClockwise(motorID, 0);
  }
}

/**
 * Drives a given motor at a given speed in a clockwise direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveClockwise(int motorID, int speed){  
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

void setup()
{
  // Initialize the serial connection to the saberteeth
  Serial3.begin(9600);
  // Stop all the motors, if they were moving.
  // This must be done AFTER stopping all the motors.
  stopAllMotors(); 
}

void loop()
{
  
}


