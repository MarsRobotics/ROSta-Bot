

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
#include <command2ros/ManualCommand.h>
#include <EncoderFL.h>
#include <EncoderML.h>
#include <EncoderFR.h>
#include <EncoderMR.h>
#include <EncoderRL.h>
#include <EncoderRR.h>
#include "SabertoothDriverROS.h"

std_msgs::String msg;
command2ros::ManualCommand currentRotationMessage
ros::Publisher pubResults("testing", &msg);
ros::Publisher pubCurrentRotation("Current Rotation", &currentRotationMessage)// current rotation data for each wheel. 
                                                                              // This will publish when the angle is updated.
EncoderFL efl;
EncoderML eml;
EncoderFR efr;
EncoderMR emr;
EncoderRL erl;
EncoderRR err;
// Have we done a software E-stop?
// By default, we have (this way the robot won't move until we get 
// valid control input)
bool emergencyStop = true;

// Is the robot currently rotating?
// If true, we should override motor drive velocity with rotation velocity.
bool currentlyRotating = false;
bool currentlyDriving = false;
bool currentlyStopped = false;

const int ARTICULATION_OFFSET = 6;
// How long should we drive for (milliseconds)?
unsigned long driveTimeMillis = 0;
//DriveUntil: Used when driving forwards.
// Should be of the form millis() + driveTimeMillis
unsigned long driveUntilTime = 0;


// By default, the velocity is zero.
char current_velocity = 0;
Wheel_status targetWheelStatus[6];
Wheel_status currentWheelStatus[6];

void newManualCommandCallback(const command2ros::ManualCommand& nmc)
{
  char* error = "received command";
  msg.data = error;
  pubResults.publish(&msg);
  
  // TODO: Update wheel status info. Determine if we need to rotate.
  currentlyRotating = currentlyRotating || updateTargetWheelStatus(nmc);
  if(currentlyRotating)
  {
    char* error = "currentlyRotating is TRUE.";
    msg.data = error;
    pubResults.publish(&msg);  
  }
  else
  {
    char* error = "currentlyRotating is FALSE.";
    msg.data = error;
    pubResults.publish(&msg);   
  }
  driveTimeMillis = (long)(nmc.drive_duration * 1000);
  emergencyStop = nmc.e_stop;
  
  if(emergencyStop == true)
  {
    char* error = "Setting E-Stop to TRUE.";
    msg.data = error;
    pubResults.publish(&msg);  
  }
  else
  {
    char* error = "Setting E-Stop to FALSE.";
    msg.data = error;
    pubResults.publish(&msg);   
  }
  
}


// Function to update wheel status variables. 
// Returns TRUE if an orientation changed, FALSE otherwise.
bool updateTargetWheelStatus(const command2ros::ManualCommand& nmc)
{
  bool retval = false;
  // Check wheel orientations:
  // Check if the front-left wheel orientation should be changed.
  if(targetWheelStatus[FRONT_LEFT_DRIVE_MOTOR_ID].orientation != (int)nmc.fl_articulation_angle)
  {
    retval = true;
    targetWheelStatus[FRONT_LEFT_DRIVE_MOTOR_ID].orientation = (int)nmc.fl_articulation_angle;
  }

  // Check if the middle-left wheel orientation should be changed.
  if(targetWheelStatus[MIDDLE_LEFT_DRIVE_MOTOR_ID].orientation != (int)nmc.ml_articulation_angle)
  {
    retval = true;
    targetWheelStatus[MIDDLE_LEFT_DRIVE_MOTOR_ID].orientation = (int)nmc.ml_articulation_angle;
  }

  // Check if the rear-left wheel orientation should be changed.
  if(targetWheelStatus[REAR_LEFT_DRIVE_MOTOR_ID].orientation != (int)nmc.rl_articulation_angle)
  {
    retval = true;
    targetWheelStatus[REAR_LEFT_DRIVE_MOTOR_ID].orientation = (int)nmc.rl_articulation_angle;
  }

  // Check if the front-right wheel orientation should be changed.
  if(targetWheelStatus[FRONT_RIGHT_DRIVE_MOTOR_ID].orientation != (int)nmc.fr_articulation_angle)
  {
    retval = true;
    targetWheelStatus[FRONT_RIGHT_DRIVE_MOTOR_ID].orientation = (int)nmc.fr_articulation_angle;
  }

  // Check if the middle-right wheel orientation should be changed.
  if(targetWheelStatus[MIDDLE_RIGHT_DRIVE_MOTOR_ID].orientation != (int)nmc.mr_articulation_angle)
  {
    retval = true;
    targetWheelStatus[MIDDLE_RIGHT_DRIVE_MOTOR_ID].orientation = (int)nmc.mr_articulation_angle;
  }

  // Check if the rear-right wheel orientation should change.
  if(targetWheelStatus[REAR_RIGHT_DRIVE_MOTOR_ID].orientation != (int)nmc.rr_articulation_angle)
  {
    retval = true;
    targetWheelStatus[REAR_RIGHT_DRIVE_MOTOR_ID].orientation = (int)nmc.rr_articulation_angle;
  }

  // Update target drive speeds.
  targetWheelStatus[FRONT_LEFT_DRIVE_MOTOR_ID].velocity = (long)nmc.fl_drive_speed;
  targetWheelStatus[FRONT_RIGHT_DRIVE_MOTOR_ID].velocity = (long)nmc.fr_drive_speed;
  targetWheelStatus[MIDDLE_LEFT_DRIVE_MOTOR_ID].velocity = (long)nmc.ml_drive_speed;
  targetWheelStatus[MIDDLE_RIGHT_DRIVE_MOTOR_ID].velocity = (long)nmc.mr_drive_speed;
  targetWheelStatus[REAR_LEFT_DRIVE_MOTOR_ID].velocity = (long)nmc.rl_drive_speed;
  targetWheelStatus[REAR_RIGHT_DRIVE_MOTOR_ID].velocity = (long)nmc.rr_drive_speed;
  return retval;

}



// Function to drive the motors as specified in the ROS packet.
// HANDY VALUES:
// speed: unsigned char representing the velocity (0-127).
void continueDriving(){
  // Short-circuit on EStop!
  if(emergencyStop)
  { 
    if (!currentlyStopped) {
      char* error = "E-Stopped all motors.";
      msg.data = error;
      pubResults.publish(&msg);  
      
    }
    stopAllMotors();
   
    // short-circuit
    return;
  }

  if(currentlyRotating)
  {
    // TODO:Rotate 
    articulate();
  }
  else
  {
    // If we still intend to drive, do so.
    if(millis() < driveUntilTime)
    {
      if(!currentlyDriving)
      {
        // TODO: Drive
        driveAllMotors();
        currentlyDriving = true;
      }
    }
    else
    {
      currentlyDriving = false;
      char* error = "done: Stopped all motors.";
      msg.data = error;
      pubResults.publish(&msg);  
      
      stopAllMotors();
    }
  }

}

bool currentlyMoving[6];
// A function to articulate the wheels to face the desired orientation.
void articulate()
{
  // For now, only articulate the FRONT LEFT wheel.
  int delta = 0;
  int motorIDX;
 
  bool stillRotating = false;

  for(motorIDX = 0; motorIDX <= 5; ++motorIDX)
  {
    // Which direction should we rotate?
    delta  = targetWheelStatus[motorIDX].orientation - currentWheelStatus[motorIDX].orientation;
    if(delta < 3 && delta > -3)
    {
      driveCounterclockwise(0, motorIDX + ARTICULATION_OFFSET);
      driveCounterclockwise(0, motorIDX);

      char* error = "Stopping motor: X";
      error[16] = '0'+motorIDX;
      msg.data = error;
      pubResults.publish(&msg);
      // TODO: update this when we have multi-wheel support
      currentlyMoving[motorIDX] = false;
    }
    else if (((delta > 0) && (delta < 180)) || 
      ((delta + 360) < 180 ))
    {
      stillRotating = true;
      if(!currentlyMoving[motorIDX])
      {
        char* error = "Starting motor: X";
        currentlyStopped = false;
        error[16] = '0'+motorIDX;
        msg.data = error;
        pubResults.publish(&msg);
        driveCounterclockwise(ARTICULATION_SPEED, motorIDX + ARTICULATION_OFFSET);
        driveCounterclockwise(ARTICULATION_DRIVE_SPEED, motorIDX);
        currentlyMoving[motorIDX] = true;
      }

    }
    else
    {
      if(!currentlyMoving[motorIDX])
      {
        char* error = "Starting motor: X";
        currentlyStopped = false;
        error[16] = '0'+motorIDX;
        msg.data = error;
        pubResults.publish(&msg);
        driveClockwise(ARTICULATION_SPEED, motorIDX + ARTICULATION_OFFSET);
        driveClockwise(ARTICULATION_DRIVE_SPEED, motorIDX);
        stillRotating = true;
        currentlyMoving[motorIDX] = true;
      }
    }
  }

  if(!stillRotating)
  {
    driveUntilTime = millis() + driveTimeMillis;
    currentlyRotating = false;
  }
  else
  {
    currentlyRotating = true;
  }

}

// A function that tells all motors to stop moving.
void stopAllMotors()
{
  if (!currentlyStopped) {
    currentlyStopped = true;
    // send stop-driving commands to all motors
    for(int i = 0; i < 12; ++i)
    {
      driveClockwise(0,i);
    }
    char* error = "Stopping all motors.";
    msg.data = error;
    pubResults.publish(&msg);  
  }
}

// A function to drive all motors at the desired rates.
void driveAllMotors()
{
  // send driving commands to all motors
  for(int i = 0; i < 6; ++i)
  {
    driveClockwise(targetWheelStatus[i].velocity, i);
  }
}

// Function to drive a specific motor forward. 
// Packet format: Address Byte, Command Byte, Value Byte, Checksum.
void driveClockwise(char speed, char motor){
  // Build the data packet:
  // Get the address and motor command ID from a predefined array.
  unsigned char address = MOTOR_ADDRESS[motor];
  unsigned char command = MOTOR_COMMAND[motor];
  unsigned char checksum = (address + command + speed) & 0b01111111;
  // Write the packet.
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(speed);
  Serial3.write(checksum);
  delayMicroseconds(1000);
}

// Function to drive a specific motor backwards. 
// Packet format: Address Byte, Command Byte, Value Byte, Checksum.
void driveCounterclockwise(char speed, char motor){
  unsigned char address = MOTOR_ADDRESS[motor];
  unsigned char command = MOTOR_COMMAND[motor] + 1;
  unsigned char checksum = (address + command + speed) & 0b01111111;
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(speed);
  Serial3.write(checksum);
  delayMicroseconds(1000);
}

// This node handle represents this arduino. 
ros::NodeHandle sabertoothDriverNode;
// Subscribers for intended drive velocity and direction
ros::Subscriber<command2ros::ManualCommand> mcSUB("ManualCommand", &newManualCommandCallback );

/**
* Gets the last known position of each articulation joint, and updates acordingly.
*/
void updateArticulationValues()
{
  int temp = EncoderFL::getPosition();
  if (temp < 0)
  {
    temp += 400;
  }
  currentWheelStatus[FRONT_LEFT_DRIVE_MOTOR_ID].orientation = (int)((temp * 9L) / 10L);
  currentRotationMessage.FRONT_LEFT_articulation_angle = currentWheelStatus[FRONT_LIGHT_DRIVE_MOTOR_ID].orientation;

  temp = EncoderML::getPosition();
  if (temp < 0)
  {
    temp += 400;
  }
  currentWheelStatus[MIDDLE_LEFT_DRIVE_MOTOR_ID].orientation = (int)((temp * 9L) / 10L);
  currentRotationMessage.ml_articulation_angle = currentWheelStatus[MIDDLE_LEFT_DRIVE_MOTOR_ID].orientation;

  temp = EncoderRL::getPosition();
  if (temp < 0)
  {
    temp += 400;
  }
  currentWheelStatus[REAR_LEFT_DRIVE_MOTOR_ID].orientation = (int)((temp * 9L) / 10L);
  currentRotationMessage.rl_articulation_angle = currentWheelStatus[REAR_LEFT_DRIVE_MOTOR_ID].orientation;

  temp = EncoderFR::getPosition() - 200; // Subtract 200 b/c the right side is flipped.
  if (temp < 0)
  {
    temp += 400;
  }
  currentWheelStatus[FRONT_RIGHT_DRIVE_MOTOR_ID].orientation = (int)((temp * 9L) / 10L);
  currentRotationMessage.fr_articulation_angle = currentWheelStatus[FRONT_RIGHT_DRIVE_MOTOR_ID].orientation;

  temp = EncoderMR::getPosition() - 200;
  if (temp < 0)
  {
    temp += 400;
  }
  currentWheelStatus[MIDDLE_RIGHT_DRIVE_MOTOR_ID].orientation = (int)((temp * 9L) / 10L);
  currentRotationMessage.mr_articulation_angle = currentWheelStatus[MIDDLE_RIGHT_DRIVE_MOTOR_ID].orientation;

  temp = EncoderRR::getPosition() - 200;
  if (temp < 0)
  {
    temp += 400;
  }

  currentWheelStatus[REAR_RIGHT_DRIVE_MOTOR_ID].orientation = (int)((temp * 9L) / 10L);
  currentRotationMessage.rr_articulation_angle = currentWheelStatus[REAR_RIGHT_DRIVE_MOTOR_ID].orientation;

  pubCurrentRotation.publish(&currentRotationMessage);
}

/**
* Is called on starting the arduino.
* 
*/
void setup(){

  // Communicate with the computer
  EncoderFL::setupEncoderFL();
  EncoderML::setupEncoderML();
  EncoderRL::setupEncoderRL();
  EncoderFR::setupEncoderFR();
  EncoderMR::setupEncoderMR();
  EncoderRR::setupEncoderRR();

  sabertoothDriverNode.initNode();
  sabertoothDriverNode.subscribe(mcSUB);

  // Communicate with the Sabertooth
  Serial3.begin(9600);

  // Initialize the message
  currentRotationMessage.fl_articulation_angle = 0;
  currentRotationMessage.fr_articulation_angle = 180;
  currentRotationMessage.ml_articulation_angle = 0;
  currentRotationMessage.mr_articulation_angle = 180;
  currentRotationMessage.rl_articulation_angle = 0;
  currentRotationMessage.rr_articulation_angle = 180;
  
  currentRotationMessage.fl_drive_speed = 0;
  currentRotationMessage.fr_drive_speed = 0;
  currentRotationMessage.ml_drive_speed = 0;
  currentRotationMessage.mr_drive_speed = 0;
  currentRotationMessage.rl_drive_speed = 0;
  currentRotationMessage.rr_drive_speed = 0;

  // Initialize current wheel status: Assume we're in "closed" position
  currentWheelStatus[FRONT_LEFT_DRIVE_MOTOR_ID].orientation = 0.0;     //.fl_articulation_angle = 0.0;
  currentWheelStatus[FRONT_RIGHT_DRIVE_MOTOR_ID].orientation = 180.0;   //.fr_articulation_angle = 180.0;
  currentWheelStatus[MIDDLE_LEFT_DRIVE_MOTOR_ID].orientation = 0.0;//.ml_articulation_angle = 0.0;
  currentWheelStatus[MIDDLE_RIGHT_DRIVE_MOTOR_ID].orientation = 180.0;//.mr_articulation_angle = 180.0;
  currentWheelStatus[REAR_LEFT_DRIVE_MOTOR_ID].orientation = 0.0;//.rl_articulation_angle = 0.0;
  currentWheelStatus[REAR_RIGHT_DRIVE_MOTOR_ID].orientation = 180.0; //.rr_articulation_angle = 180.0;
  
  currentWheelStatus[FRONT_LEFT_DRIVE_MOTOR_ID].velocity = 0.0;//.fl_drive_speed = 0.0;
  currentWheelStatus[FRONT_RIGHT_DRIVE_MOTOR_ID].velocity = 0.0;//.fr_drive_speed = 0.0;
  currentWheelStatus[MIDDLE_LEFT_DRIVE_MOTOR_ID].velocity = 0.0;//.ml_drive_speed = 0.0;
  currentWheelStatus[MIDDLE_RIGHT_DRIVE_MOTOR_ID].velocity = 0.0;//.mr_drive_speed = 0.0;
  currentWheelStatus[REAR_LEFT_DRIVE_MOTOR_ID].velocity = 0.0;//.rl_drive_speed = 0.0;
  currentWheelStatus[REAR_RIGHT_DRIVE_MOTOR_ID].velocity = 0.0;//.rr_drive_speed = 0.0;

  // Publish our stuff
  sabertoothDriverNode.advertise(pubResults);
  sabertoothDriverNode.advertise(pubCurrentRotation);
}

// This program does whatever ROS directs it to do.
// So far, it drives the robot forwards and backwards.
void loop(){
  delayMicroseconds(100000);
  sabertoothDriverNode.spinOnce(); // Check for subscriber update/update timestamp
  updateArticulationValues();
  continueDriving();
  // TODO: Delete this?
  //  msg.data = (long)currentWheelStatus[FRONT_LEFT_DRIVE_MOTOR_ID].orientation;
  //  pubResults.publish(&msg);

}


