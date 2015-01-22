from src.transporter.src import transport_drive_motor_API

__author__ = 'Matt Delaney'
from src.state_machine.src.state_machine import *
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

# Is the robot close enough to the target distance? Do we want to stop?
def close_enough():
    return abs(targetDistance - currentDistance) < MARGIN_OF_ERROR

# Is the target too far away? Do we want to drive towards the target?
def target_too_far_away():
    return targetDistance > (currentDistance + MARGIN_OF_ERROR)

# Is the target too close? Do we want to drive away from the target?
def target_too_close():
    return targetDistance < (currentDistance - MARGIN_OF_ERROR)


# Initialize the wooden-robot state machine.
woodenStateMachine = StateMachine()
# Initialize the transport drive motor command center (API).
robot = transport_drive_motor_API.TransportDriveMotorAPI()
# The start state is the state where the robot does not move.
start = State("start", lambda: robot.do_not_move)
# The other two states have the robot drive backwards and forwards, respectively.
drivingBackwards = State("driving backwards", lambda: robot.simple_drive_forwards)
drivingForwards = State("driving forwards", lambda: robot.simple_drive_backwards)
# If the robot is "close enough" to the target distance, stay put.
start.addTransition("close enough: No movement necessary", lambda: close_enough, start)
# If the target is too far away, drive towards it.
start.addTransition("Vehicle too far away", lambda: target_too_far_away, drivingForwards)
# If the target is too close, back up.
start.addTransition("Vehicle too close to target", lambda: target_too_close, drivingBackwards)

# Stop moving if we're moving in the wrong direction.
drivingForwards.addTransition("Stop moving towards the target", lambda: close_enough() or target_too_close(), start)
drivingForwards.addTransition("Stop moving away from the target", lambda: close_enough() or target_too_close(), start)

# All measurements are in Centimeters.
# How far away from the target do we want to be? (cm)
targetDistance = 200
# How far away from the target are we, currently? (cm)
currentDistance = 200
# How far away from the target can we be and still be 'close enough'? (cm)
MARGIN_OF_ERROR = 10
