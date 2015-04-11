#!/usr/bin/env python
__author__ = 'Matt Delaney'

import sys
import rospy
from std_msgs.msg import Int64
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
        start = self.woodenStateMachine.addState("start", lambda: self.robot.do_not_move)
        # set the start state and current state to the new start state.
        self.woodenStateMachine.startState = start
        self.woodenStateMachine.currentState = start
        # The other two states have the robot drive backwards and forwards, respectively.
        drivingBackwards = self.woodenStateMachine.addState("driving backwards", lambda: self.robot.simple_drive_forwards)
        drivingForwards = self.woodenStateMachine.addState("driving forwards", lambda: self.robot.simple_drive_backwards)
        # If the robot is "close enough" to the target distance, stay put.
        start.addTransition("close enough: No movement necessary", lambda: self.close_enough, start)
        # If the target is too far away, drive towards it.
        start.addTransition("Vehicle too far away", lambda: self.target_too_far_away, drivingForwards)
        # If the target is too close, back up.
        start.addTransition("Vehicle too close to target", lambda: self.target_too_close, drivingBackwards)
        # Stop moving if we're moving in the wrong direction.
        drivingForwards.addTransition("Stop moving towards the target", lambda: self.close_enough() or self.target_too_close(), start)
        drivingForwards.addTransition("Stop moving away from the target", lambda: self.close_enough() or self.target_too_close(), start)

        # All measurements are in Centimeters.
        # How far away from the target do we want to be? (cm)
        self.targetDistance = 100
        # How far away from the target are we, currently? (cm)
        self.currentDistance = 100
        # How far away from the target can we be and still be 'close enough'? (cm)
        self.MARGIN_OF_ERROR = 10
        # Subscribe to the "target distance" topic (note that this topic may not be active / may need to be
        # manually set).
        #rospy.init_node("simple_state_machine")
        self.target_distance_subscriber = rospy.Subscriber("target_distance", Int64, lambda: self.target_distance_changed)
        # Subscribe to the current distance from the target. For now, that's an IR value.
        self.current_distance_subscriber = rospy.Subscriber("range_data", position, lambda: self.current_distance_reading_changed)
	
        #spin for all eternity. Note that, in Python, each ROS callback NEEDS TO TICK THE STATE MACHINE.
        rospy.spin()

    # Is the robot close enough to the target distance? Do we want to stop?
    def close_enough(self):
        return abs(self.targetDistance - self.currentDistance) < self.MARGIN_OF_ERROR

    # Is the target too far away? Do we want to drive towards the target?
    def target_too_far_away(self):
        return self.targetDistance > (self.currentDistance + self.MARGIN_OF_ERROR)

    # Is the target too close? Do we want to drive away from the target?
    def target_too_close(self):
        return self.targetDistance < (self.currentDistance - self.MARGIN_OF_ERROR)

    # function callback for when the target distance is changed.
    # this should, for the time being, be a value between 30cm and 150cm.
    # Once the distance is not IR-dependent, this range can expand.
    def target_distance_changed(self, new_target_distance):
        self.targetDistance = new_target_distance.data
        # Data has changed! Tick the state machine!
        ssm.woodenStateMachine.tick()

    def current_distance_reading_changed(self, new_current_distance):
        self.currentDistance = new_current_distance.x_distance
	print self.currentDistance
        # Data has changed! Tick the state machine!
        ssm.woodenStateMachine.tick()


print("compiled!")
ssm = SimpleStateMachine()
