#!/usr/bin/env python
__author__ = 'Matt Delaney'

import sys
import rospy
from std_msgs.msg import Int64, Int32
sys.path.append("/home/pi/ROSta-Bot/src/transporter/src")
from transport_drive_motor_API import *
from state_machine import *
from driving_interface.msg import position

# This is a simple 3-state machine. It moves the robot to a predetermined distance from the bin.
#
#
#
#  -----------
#  -    sensor->
#  -   faces
#  -  forwards
#  -----------
#  (Forwards ->)
#
class SimpleStateMachine:

    def __init__(self):
        # Initialize the wooden-robot state machine.
        self.woodenStateMachine = StateMachine()
        # Initialize the transport drive motor command center (API).
        self.robot = TransportDriveMotorAPI()
        # The start state is the state where the robot does not move.
        start = self.woodenStateMachine.addState("not moving", self.robot.do_not_move)
        rotating = self.woodenStateMachine.addState("rotating to face a specific direction", self.rotateToFace)
        # set the start state and current state to the new start state.
        self.woodenStateMachine.startState = start
        self.woodenStateMachine.currentState = start
        # If the robot is "close enough" to the target angle, stay put.
        start.addTransition("close enough: No movement necessary", self.should_robot_restart, start)
        start.addTransition("Angle is bad: Rotating.", self.check_robot_angle, rotating)

        rotating.addTransition("Angle is bad: Continue Rotating", self.check_robot_angle, rotating)
        rotating.addTransition("Angle is close enough: Stop Rotating", self.should_robot_stop, start)
        # All measurements are in Centimeters.
        # At what angle to the target do we want to be? (degrees)
        self.targetAngle = 0
        # How far away from the target are we, currently? (degrees)
        self.currentRobotOrientation = 0
        # How far away from the target can we be and still be 'close enough'? (degrees)
        self.STOP_AT = 10
        self.RESTART_AT = 15
        self.CLOCKWISE = -1
        self.NOT_MOVING = 0
        self.COUNTERCLOCKWISE = 3

        self.currentCameraAngle = 0
        self.currentPosition = position()
        self.currentPosition.yPose = 180
        # Current direction of rotation
        self.current_rotation_direction = self.NOT_MOVING
        # Subscribe to the "target robot angle" topic (note that this topic may not be active / may need to be
        # manually set).
        #rospy.init_node("simple_state_machine")
        self.target_angle_subscriber = rospy.Subscriber("target_robot_angle", Int64, self.target_angle_changed)
        # Subscribe to the current distance from the target. For now, that's an IR value.
        self.current_angle_subscriber = rospy.Subscriber("range_data", position, self.current_position_reading_changed)
        self.current_camera_angle_subscriber = rospy.Subscriber("current_camera_angle", Int32, self.current_camera_angle_changed )
        #spin for all eternity. Note that, in Python, each ROS callback NEEDS TO TICK THE STATE MACHINE.
        rospy.spin()

    # Is the robot not close enough to the target distance? Do we want to relocate?
    def should_robot_restart(self):
        return abs(self.targetAngle - self.currentRobotOrientation) < self.RESTART_AT

    # Should the robot stay put?
    def should_robot_stop(self):
        #TODO update this!
        return abs(self.targetAngle - self.currentRobotOrientation) < self.STOP_AT

    def rotate_to_face(self):
        self.currentRobotOrientation = 180 - self.currentPosition.yPose - self.currentCameraAngle
        if (self.currentRobotOrientation - self.targetAngle) > 0:
            # Rotate Clockwise
            if self.current_rotation_direction != self.CLOCKWISE:
                self.current_rotation_direction = self.CLOCKWISE
                self.robot.simple_turn_direction(self.CLOCKWISE)
        else:
            #Rotate Counterclockwise
            if self.current_rotation_direction != self.COUNTERCLOCKWISE:
                self.current_rotation_direction = self.COUNTERCLOCKWISE
                self.robot.simple_turn_direction(self.COUNTERCLOCKWISE)


    # function callback for when the target distance is changed.
    # this should, for the time being, be a value between 30cm and 150cm.
    # Once the distance is not IR-dependent, this range can expand.
    def target_angle_changed(self, new_target_angle):
        self.targetAngle = new_target_angle.data
        # Data has changed! Tick the state machine!
        self.woodenStateMachine.tick()

    def current_position_reading_changed(self, new_position):
        self.currentPosition = new_position
        # Data has changed! Tick the state machine!
        self.woodenStateMachine.tick()

    def current_camera_angle_changed(self, new_camera_angle):
        self.currentCameraAngle = new_camera_angle.data
        self.woodenStateMachine.tick()
ssm = SimpleStateMachine()
