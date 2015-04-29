//
//  Conveyor.c
//  
//
//  Created by Matt on 4/27/15.
//
//This may change to be a different ROS message.
#include <command2ros/ManualCommand.h>


// For now, only the right conveyor motor is actually hooked up.
const unsigned char LEFT_CONVEYOR_MOTOR_ID = 12;
const unsigned char RIGHT_CONVEYOR_MOTOR_ID = 13;
const unsigned char WINCH_MOTOR_ID = 14;


const unsigned char LEFT_CONVEYOR_MOTOR_ADDRESS = 999;
const unsigned char RIGHT_CONVEYOR_MOTOR_ADDRESS = 131;
const unsigned char WINCH_MOTOR_ADDRESS = 131;

const unsigned char LEFT_CONVEYOR_MOTOR_COMMAND = 9;
const unsigned char RIGHT_CONVEYOR_MOTOR_COMMAND = 4;
const unsigned char WINCH_MOTOR_COMMAND = 0;

const unsigned char LEFT_CONVEYOR_MOTOR_FLIPPED = 0;
const unsigned char RIGHT_CONVEYOR_MOTOR_FLIPPED = 0;
const unsigned char WINCH_MOTOR_FLIPPED = 0;

