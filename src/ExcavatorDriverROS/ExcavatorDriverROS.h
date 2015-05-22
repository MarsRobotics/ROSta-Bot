// Header file for the SabertoothDriverROS class.
//#include <command2ros/ExcavatorCommand.h>

// Device constants
const int NUM_EXCAVATOR_MOTORS = 4;
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
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char BACK_LEFT_DRIVE_MOTOR_FLIPPED = 1;
const unsigned char BACK_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char MOTOR_FLIPPED[4] = {
    FRONT_LEFT_DRIVE_MOTOR_FLIPPED,
    FRONT_RIGHT_DRIVE_MOTOR_FLIPPED,
    BACK_LEFT_DRIVE_MOTOR_FLIPPED,
    BACK_RIGHT_DRIVE_MOTOR_FLIPPED};

const unsigned char CONVEYOR_MOTOR_ADDRESS = 132;
const unsigned char CONVEYOR_MOTOR_COMMAND = 4;

const unsigned char BUCKET_WHEEL_MOTOR_COMMAND_0 = 0;
const unsigned char BUCKET_WHEEL_MOTOR_COMMAND_1 = 0;
const unsigned char BUCKET_WHEEL_MOTOR_COMMAND_2 = 0;
const unsigned char BUCKET_WHEEL_MOTOR_COMMAND_3 = 4;
const unsigned char ROTATE_MOTOR_COMMAND[5] = {
    BUCKET_WHEEL_MOTOR_COMMAND_0,
    BUCKET_WHEEL_MOTOR_COMMAND_1,
    BUCKET_WHEEL_MOTOR_COMMAND_2,
    BUCKET_WHEEL_MOTOR_COMMAND_3,
    CONVEYOR_MOTOR_COMMAND};
      
const unsigned char BUCKET_WHEEL_MOTOR_ADDRESS_0 = 128;
const unsigned char BUCKET_WHEEL_MOTOR_ADDRESS_1 = 131;
const unsigned char BUCKET_WHEEL_MOTOR_ADDRESS_2 = 130;
const unsigned char BUCKET_WHEEL_MOTOR_ADDRESS_3 = 130;
const unsigned char ROTATE_MOTOR_ADDRESS[5] = {
  BUCKET_WHEEL_MOTOR_ADDRESS_0,
  BUCKET_WHEEL_MOTOR_ADDRESS_1,
  BUCKET_WHEEL_MOTOR_ADDRESS_2,
  BUCKET_WHEEL_MOTOR_ADDRESS_3,
  CONVEYOR_MOTOR_ADDRESS};
  
const unsigned char CONVEYOR_MOTOR_ID = 4;