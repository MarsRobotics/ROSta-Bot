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
#include <command2ros/ManualCommand.h>
#include <SabertoothDriverROS.h>

// Have we done a software E-stop?
// By default, we have (this way the robot won't move until we get 
// valid control input)
bool emergencyStop = true;

// Is the robot currently rotating?
// If true, we should override motor drive velocity with rotation velocity.
bool currentlyRotating = false;

// How long should we drive for (milliseconds)?
unsigned long driveTimeMillis = 0;
//DriveUntil: Used when driving forwards.
// Should be of the form millis() + driveTimeMillis
unsigned long driveUntilTime = 0;


// By default, the velocity is zero.
char current_velocity = 0;
Wheel_status[6] targetWheelStatus;
Wheel_status[6] currentWheelStatus;

void newManualCommandCallback(const command2ros::ManualCommand& nmc)
{
 // TODO: Update wheel status info. Determine if we need to rotate.
 currentlyRotating = currentlyRotating || updateTargetWheelStatus(nmc);
 driveTimeMillis = (long)(nmc.drive_duration * 1000);
 emergencyStop = nmc.emergencyStop;
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
    // TODO: Drive
    }
    else
    {
      stopAllMotors();
    }
  }
  
}

// A function to articulate the wheels to face the desired orientation.
void articulate()
{
  // For now, only articulate the FRONT LEFT wheel.
  int delta = 0;
  
  // Which direction should we rotate?
  delta  = targetWheelStatus.fl_articulation_angle - currentWheelStatus.fl_articulation_angle;
  if(0==delta)
  {
    // TODO: update this when we have multi-wheel support
    rotating = false;
  }
  else if (((delta > 0) && (delta < 180)) || 
  ((delta + 360) < 180 ))
  {
    driveCounterclockwise(ARTICULATION_SPEED, FRONT_LEFT_ARTICULATION_MOTOR);
    driveCounterclockwise(ARTICULATION_DRIVE_SPEED, FRONT_LEFT_DRIVE_MOTOR);
  }
  else
  {
    driveClockwise(ARTICULATION_SPEED, FRONT_LEFT_ARTICULATION_MOTOR);
    driveClockwise(ARTICULATION_DRIVE_SPEED, FRONT_LEFT_DRIVE_MOTOR);
  }
  
  
}

// A function that tells all motors to stop moving.
void stopAllMotors()
{
    // send stop-driving commands to all motors
    for(int i = 0; i < 12; ++i)
    {
      driveClockwise(0,i);
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
}

// This node handle represents this arduino. 
ros::NodeHandle sabertoothDriverNode;
// Subscribers for intended drive velocity and direction
ros::Subscriber<command2ros::ManualCommand> mcSUB("???", newManualCommandCallback );


void setup(){
  // Communicate with the computer
  sabertoothDriverNode.initNode();
  sabertoothDriverNode.subscribe(mcSUB);
  // Communicate with the Sabertooth
  Serial3.begin(9600);
  
  // Initialize current wheel status: Assume we're in "closed" position
  currentWheelStatus.fl_articulation_angle = 0.0;
  currentWheelStatus.fr_articulation_angle = 180.0;
  currentWheelStatus.ml_articulation_angle = 0.0;
  currentWheelStatus.mr_articulation_angle = 180.0;
  currentWheelStatus.rl_articulation_angle = 0.0;
  currentWheelStatus.rr_articulation_angle = 180.0;
  currentWheelStatus.fl_drive_speed = 0.0;
  currentWheelStatus.fr_drive_speed = 0.0;
  currentWheelStatus.ml_drive_speed = 0.0;
  currentWheelStatus.mr_drive_speed = 0.0;
  currentWheelStatus.rl_drive_speed = 0.0;
  currentWheelStatus.rr_drive_speed = 0.0;
}

// This program does whatever ROS directs it to do.
// So far, it drives the robot forwards and backwards.
void loop(){
  sabertoothDriverNode.spinOnce();
  continueDriving();
  // TODO: Delete this?
  delayMicroseconds(1000);

}
