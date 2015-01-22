/* 
 / This is a simple Arduino program that demonstrates the
 / "Packetized Serial" method of using the Sabertooth Motor
 / Controller. 
 /
 / SETUP: Be sure that the Sabertooth has DIPs 4,5,6 on (1,2 off).
 /  DIP 3 is used to indicate Lithium battery mode, and will vary
 /  in its position based on the power source.
 / Sabertooth[s] should be connected on S1 to TX3 on an Arduino Mega 2560.
 / This allows it to read 9600-baud commands from the Arduino Mega.
 / Be careful to use TX3: TX1 should be reserved for Arduino-Computer
 / communications.
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>

// CONSTANTS 
// We'll track which direction we're currently moving in this demo
// So we don't have to write more packets than we have to.
const char FORWARD_LABEL = 1;
const char BACKWARD_LABEL = 2;
// Motor constants (used to communicate with the onboard computer).
const unsigned char FRONT_LEFT_DRIVE_MOTOR_ID = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
const unsigned char MIDDLE_LEFT_DRIVE_MOTOR_ID = 2;
const unsigned char MIDDLE_RIGHT_DRIVE_MOTOR_ID = 3;
const unsigned char REAR_LEFT_DRIVE_MOTOR_ID = 4;
const unsigned char REAR_RIGHT_DRIVE_MOTOR_ID = 5;

// Motor addressing (used to link the computer-constants to
// commands sent to the saberteeth).
const unsigned char FRONT_LEFT_DRIVE_MOTOR_ADDRESS = 129;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ADDRESS = 130;
const unsigned char MIDDLE_LEFT_DRIVE_MOTOR_ADDRESS = 130;
const unsigned char MIDDLE_RIGHT_DRIVE_MOTOR_ADDRESS = 129;
const unsigned char REAR_LEFT_DRIVE_MOTOR_ADDRESS = 128;
const unsigned char REAR_RIGHT_DRIVE_MOTOR_ADDRESS = 128;

// Motor addressing offset: If a motor is connected to M1, 
// This will be 0. If the motor is connected to M2, this should
// be 4.
// A value of 0 or 4 corresponds to 'forwards'. A value of 1 or 5
// corresponds to 'backwards'.
const unsigned char FRONT_LEFT_DRIVE_MOTOR_COMMAND = 4;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char MIDDLE_LEFT_DRIVE_MOTOR_COMMAND = 4;
const unsigned char MIDDLE_RIGHT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char REAR_LEFT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char REAR_RIGHT_DRIVE_MOTOR_COMMAND = 4;

// shorthand that will allow us to use the motor ID as an array index to access
// the relevant constants.
const unsigned char DRIVE_MOTOR_ADDRESS[6] = {
  FRONT_LEFT_DRIVE_MOTOR_ADDRESS,
  FRONT_RIGHT_DRIVE_MOTOR_ADDRESS,
  MIDDLE_LEFT_DRIVE_MOTOR_ADDRESS,
  MIDDLE_RIGHT_DRIVE_MOTOR_ADDRESS,
  REAR_LEFT_DRIVE_MOTOR_ADDRESS,
  REAR_RIGHT_DRIVE_MOTOR_ADDRESS};
const unsigned char DRIVE_MOTOR_COMMAND[6] = {
  FRONT_LEFT_DRIVE_MOTOR_COMMAND,
  FRONT_RIGHT_DRIVE_MOTOR_COMMAND,
  MIDDLE_LEFT_DRIVE_MOTOR_COMMAND,
  MIDDLE_RIGHT_DRIVE_MOTOR_COMMAND,
  REAR_LEFT_DRIVE_MOTOR_COMMAND,
  REAR_RIGHT_DRIVE_MOTOR_COMMAND};
  
  
// Track current motor status so we don't overload our serial lines
 
 // Do we have a new command, or is it business as usual?
 // By default, we have no new commands.
unsigned char new_commands = 0;
// By default, the robot will intend to drive forwards.
char current_direction = FORWARD_LABEL;
// By default, the velocity is zero.
char current_velocity = 0;


// Callback to handle ROS changing the intended direction of the robot.
void newDirectionCallback(const std_msgs::Char& newDir){
  // If the direction is different, 
  if (newDir.data != FORWARD_LABEL)
  {
    // Only accept valid directions (forwards or backwards)
    if((newDir.data == FORWARD_LABEL) || (newDir.data == BACKWARD_LABEL))
    {
      new_commands += 1; 
      current_direction = newDir.data;
    }
  }
  
}

// Function to process new motor speeds.
void newMotorSpeedCallback(const std_msgs::Char& newSpeed){
  // If the data is new and valid (within the 0-127 range)
  if ((newSpeed.data != current_velocity) && (newSpeed.data > 0))
  {
    new_commands += 1;
    current_velocity = newSpeed.data;
  }
}

// Function to drive all the motors forwards or backwards 
//   at the same specific velocity. 
// PARAMETERS:
// speed: unsigned char representing the velocity (0-127).
// direction: the constant char (FOWARDS_LABEL or BACKWARDS_LABEL) 
//   indicating which way we want the wheels to spin. 
void driveAllMotors(char speed, char direction){
 char i = 0;
 if (FORWARD_LABEL == direction) {
   for (i = 0; 6 > i; ++i) {
    driveForwards(speed, i);
   }
 }
 else if (BACKWARD_LABEL == direction){
   for (i = 0; 6 > i; ++i) {
    driveBackwards(speed, i);
   }   
 }
}

// Function to drive a specific motor forward. 
// Packet format: Address Byte, Command Byte, Value Byte, Checksum.
void driveForwards(char speed, char motor){
  // Build the data packet:
  // Get the address and motor command ID from a predefined array.
  unsigned char address = DRIVE_MOTOR_ADDRESS[motor];
  unsigned char command = DRIVE_MOTOR_COMMAND[motor];
  unsigned char checksum = (address + command + speed) & 0b01111111;
  // Write the packet.
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(speed);
  Serial3.write(checksum);
}

// Function to drive a specific motor backwards. 
// Packet format: Address Byte, Command Byte, Value Byte, Checksum.
void driveBackwards(char speed, char motor){
  unsigned char address = DRIVE_MOTOR_ADDRESS[motor];
  unsigned char command = DRIVE_MOTOR_COMMAND[motor] + 1;
  unsigned char checksum = (address + command + speed) & 0b01111111;
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(speed);
  Serial3.write(checksum);
}

// This node handle represents this arduino. 
ros::NodeHandle sabertoothDriverNode;
// Subscribers for intended drive velocity and direction
ros::Subscriber<std_msgs::Char> directionSUB("drive_direction", newDirectionCallback );
ros::Subscriber<std_msgs::Char> speedSUB("drive_speed", newMotorSpeedCallback);

void setup(){
  // Communicate with the computer
  sabertoothDriverNode.initNode();
  sabertoothDriverNode.subscribe(speedSUB);
  sabertoothDriverNode.subscribe(directionSUB);
  // Communicate with the Sabertooth
  Serial3.begin(9600);
}

// This program does whatever ROS directs it to do.
// So far, it drives the robot forwards and backwards.
void loop(){
  sabertoothDriverNode.spinOnce();
  if (new_commands) {
     new_commands = 0;
     driveAllMotors(current_velocity, current_direction);
  }
  delayMicroseconds(100000);

}
