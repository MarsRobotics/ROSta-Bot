#!/usr/bin/env python
import rospy, serial, time
from std_msgs.msg import Char

##
# animaticsMotorController
#
# Description: Sends commands to animatics smart motors
#
# TODO: regulate the frequency that commands can be sent to the motors.
#       That should prevent buffer overflows which cause slow response times
#       We could further improve this by detecting redundant packets
#  TODO: Find connection programatically
class animaticsMotorController:
    FORWARD_LABEL = 1
    BACKWARD_LABEL = 2
    COUNTERCLOCKWISE_LABEL = 3
    CLOCKWISE_LABEL = -1


    SPEED_SLOW = 100000
    SPEED_MEDIUM = 300000
    SPEED_TURNING = 300000

    allAddress = chr(128) + " "
    leftAddress = chr(129) + " "
    rightAddress = chr(130) + " "

    currentDir = FORWARD_LABEL
    currentSpeed = 0
    previousSpeed = 0    
    lastSpeedUpdate = 0
    lastDirUpdate = 0
    ##
    # __init__
    #
    # Description: Constructor initializes the serial connection to the motor
    #
    # Parameters:
    #  motorPath: File path of the serial port
    #
    def __init__(self, motorPath):
        #TODO: Don't hard-code this
        self.serialPort = serial.Serial('/dev/ttyUSB0')
        self.write("ZS ")           # Reset
        self.write("MV ")        # Mode Velocity
        self.write("EIGN(2) ")
        self.write("EIGN(3) ")
        self.write("ADT=100 ")      # Acceleration
        self.speed_subscriber = rospy.Subscriber("drive_speed", Char, self.speed_changed)
        self.dir_subscriber = rospy.Subscriber("drive_direction", Char, self.dir_changed)
        rospy.init_node("atp_animatics_motors")
        print "ATP Motor Controller Initialized."
        rospy.spin()

    def speed_changed(self, newSpeedData):
        newSpeed = newSpeedData.data
        # For now, tie into existing sabertooth speeds. 100+ is normal, less is slow.
        if newSpeed == 0:
            self.currentSpeed = 0
        elif newSpeed < 100:
            self.currentSpeed = self.SPEED_SLOW
        else:
            self.currentSpeed = self.SPEED_MEDIUM
            
        if newSpeed != self.lastSpeedUpdate:
            self.lastSpeedUpdate = newSpeed
            self.newDriveCommand()
    
    ##
    # dir_changed
    #
    # Description: subscriber handling function for when the intended direction changes.
    #
    # Parameters:
    #   newDir: the intended direction-of-travel constant (FORWARD_LABEL, LEFT_TURN_LABEL, etc).
    #
    def dir_changed(self, newDirData):
        newDir = newDirData.data
        if (-2 < newDir) and (5 > newDir):
            self.currentDir = newDir
            if newDir != self.lastDirUpdate:
                self.lastDirUpdate = newDir
                self.newDriveCommand()

    ##
    # newDriveCommand
    #
    # Description: sends a new drive command when the intended commands
    # have changed. Should only be called by subscriber update-handling methods.
    #
    #
    def newDriveCommand(self):
        if self.currentSpeed == self.previousSpeed:
            return
        # If the speed is zero, stop moving!
        if self.currentSpeed == 0:
            self.stopLeft()
            self.stopRight()
        # Otherwise, we want to move, probably.
        else:
            if self.currentDir == self.FORWARD_LABEL:
                self.driveLeft(self.currentSpeed)
                self.driveRight(self.currentSpeed)
            elif self.currentDir == self.BACKWARD_LABEL:
                self.driveLeft(0-self.currentSpeed)
                self.driveRight(0-self.currentSpeed)
            elif self.currentDir == self.COUNTERCLOCKWISE_LABEL:
                self.driveLeft(0-self.currentSpeed)
                self.driveRight(self.currentSpeed)
            elif self.currentDir == self.CLOCKWISE_LABEL:
                self.driveLeft(self.currentSpeed)
                self.driveRight(0-self.currentSpeed)
            # If we get a malformed drive command, stop the vehicle.
            else:
                self.stopLeft()
                self.stopRight()
        self.previousSpeed = self.currentSpeed
    ##
    # write
    #
    # Description: sends a command to the motors while ensuring
    #   the command has a single trailing whitespace
    #
    # Parameters:
    #   command: the command to send to the motors
    #
    #TODO: Priority Queue
    def write(self, command):
        command = command.strip() + " "
        # print command
        self.serialPort.write(command)
        print "Sending the following command: \'" + command + "\'"
        # Don't do it this way. Otherwise we will forget this is happening
        # when we are trying to debug it
        # time.sleep(0.1)

    ##
    # driveLeft
    #
    # Description: sets the velocity of the left wheels
    #
    # Parameters:
    #   velocity: speed for the motors to go (Units Unknown)
    #
    def driveLeft(self, velocity):

        # address the left wheels
        self.write(self.leftAddress)

        self.write("VT=" + str(velocity) + " ")
        print "____"+"VT=" + str(velocity) + " "+"____"
        self.write("G ")

        print "driving left"

    ##
    # driveRight
    #
    # Description: sets the velocity of the right wheels.
    # NOTE: these wheels are "backwards", so we must reverse the velocity.
    #
    # Parameters:
    #   velocity: speed for the motors to go (Units Unknown)
    #
    def driveRight(self, velocity):

        # address the right wheels
        self.write(self.rightAddress)
        self.write("VT=" + str(0-velocity) + " ")
        self.write("G ")

        print "driving right"

    ##
    # stopLeft
    #
    # Description: stops the left wheels
    #
    # Parameters:
    #   None
    #
    def stopLeft(self):

        # address the left wheels
        self.write(self.leftAddress)
        self.write("S ")

    ##
    # stopRight
    #
    # Description: stops the left wheels
    #
    # Parameters:
    #   None
    #
    def stopRight(self):

        # address the left wheels
        self.write(self.rightAddress)
        self.write("S ")

    ##
    # reset
    #
    # Description: Resets the motors
    #
    # Parameters:
    #   None
    #
    def reset(self):
        self.write(self.allAddress)
        self.write("ZS ")

    ##
    # close
    #
    # Description: Stops the motors and closes the serial port
    #
    # Parameters:
    #   None
    #
    def close(self):
        self.stopLeft()
        self.stopRight()
        self.serialPort.close()


# Test Code
#
# Runs the robot forward for 5 seconds and then stops
#
con = animaticsMotorController('/dev/ttyUSB0')
#con.driveLeft(100000)
#con.driveRight(100000)
#time.sleep(5)
#con.stopLeft()
#con.stopRight()
con.close()


        
        
