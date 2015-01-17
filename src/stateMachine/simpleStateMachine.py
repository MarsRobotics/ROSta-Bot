__author__ = 'Matt Delaney'
from stateMachine import *
import rospy
from src.transporter import TransportDriveMotorAPI
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

def closeEnough():
    return abs(targetDistance - currentDistance) < MARGIN_OF_ERROR

def targetTooFarAway():
    return targetDistance > (currentDistance + MARGIN_OF_ERROR)

def targetTooClose():
    return targetDistance < (currentDistance - MARGIN_OF_ERROR)


# Initialize the wooden-robot state machine.
woodenStateMachine = stateMachine()
# Initialize the transport drive motor command center (API).
robot = TransportDriveMotorAPI()
# The start state is the state where the robot does not move.
start = state("start", lambda: robot.do_not_move)
# The other two states have the robot drive backwards and forwards, respectively.
drivingBackwards = state("driving backwards", lambda: robot.simple_drive_forwards)
drivingForwards = state("driving forwards", lambda: robot.simple_drive_backwards)
# If the robot is "close enough" to the target distance, stay put.
start.addTransition("close enough: No movement necessary", lambda: closeEnough, start)
# If the target is too far away, drive towards it.
start.addTransition("Vehicle too far away", lambda: targetTooFarAway, drivingForwards)
# If the target is too close, back up.
start.addTransition("Vehicle too close to target", lambda: targetTooClose, drivingBackwards)

# Stop moving if we're moving in the wrong direction.
drivingForwards.addTransition("Stop moving towards the target", lambda: closeEnough() or targetTooClose(), start)
drivingForwards.addTransition("Stop moving away from the target", lambda: closeEnough() or targetTooClose(), start)

#TODO: define our units. For now, arbitrary margin of error.
targetDistance = 500
currentDistance = 500
MARGIN_OF_ERROR = 50
