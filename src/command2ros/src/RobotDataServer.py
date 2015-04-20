#!/usr/bin/env python

__author__ = 'Matt'

from RobotData import RobotData
import socket

import threading
from DataTransferProtocol import receiveData, sendData
from RobotData import ManualControlData
import rospy

import sys
sys.path.append("/home/pi/ROSta-Bot/src/command2ros/src")
from command2ros.msg import ManualCommand
import roslib
roslib.load_manifest('command2ros')

import time

# In Hertz
sendRate = 10

##
# robotDataDistributor
#
# Description: Thread that accepts network connections from clients and spawns a new thread to handle the connection
#
class robotDataDistributor(threading.Thread):

    def __init__(self):
        self.data = RobotData()
        threading.Thread.__init__(self)
        return

    def run(self):
        # configure the socket to receive incoming sockets
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('192.168.1.137', 10000)
        s.bind(server_address)
        s.listen(1)

        # Until the server closes, accept connections and spawn a thread to handle them
        while True:
            (clientsocket, address) = s.accept()
            print "Received connection from: " + address[0]
            cs = robotDataServer(clientsocket, self, address)
            cs.run()

        return


##
# robotDataServer
#
# Description: Handles a connection with a particular client. Receives commands and sends data about the robot.
#
class robotDataServer(threading.Thread):

    def __init__(self, sock, distributor, address):

        self.socket = sock
        self.distributor = distributor
        self.address = address

        threading.Thread.__init__(self)

        return

    def run(self):
        try:
            sendTime = 0
            while True:

                self.socket.setblocking(1)
                if sendTime < time.time():
                # Send the robot data to the client
                    sendData(self.socket, self.distributor.data)
                    sendTime = time.time() + 1/float(sendRate)
                    print sendTime
                # An extra exception because we have a non-blocking socket
                try:
                    self.socket.setblocking(0)

                    # Receive a command and add it to the command queue
                    newCommand = receiveData(self.socket)
                    if newCommand.e_stop:
                        commandQueue.insert(0, newCommand)
                    else:
                        commandQueue.append(newCommand)

                    print "received command"

                    # print manualControlCommand.go_forward

                except socket.error:
                    continue

        except socket.error as e:
            print "Lost connection with " + self.address[0]

            # Add a E-STOP command
            newCommand = ManualControlData()
            newCommand.e_stop = True
            commandQueue.insert(0, newCommand)
            return

        return


commandQueue = []
rdd = robotDataDistributor()
rdd.start()


pub = rospy.Publisher('ManualCommand', ManualCommand, queue_size=10)
rospy.init_node('command2ros', anonymous=True)

print "About to start..."

while True:

    # Process a command
    if len(commandQueue) > 0:
        command = commandQueue.pop(0)
	print "processing command"

        mc = ManualCommand()
        mc.fl_articulation_angle = command.fl_articulation_angle
        mc.fr_articulation_angle = command.fr_articulation_angle
        mc.ml_articulation_angle = command.ml_articulation_angle
        mc.mr_articulation_angle = command.mr_articulation_angle
        mc.rl_articulation_angle = command.rl_articulation_angle
        mc.rr_articulation_angle = command.rr_articulation_angle

        mc.fl_drive_speed = command.fl_drive_speed
        mc.fr_drive_speed = command.fr_drive_speed
        mc.ml_drive_speed = command.ml_drive_speed
        mc.mr_drive_speed = command.mr_drive_speed
        mc.rl_drive_speed = command.rl_drive_speed
        mc.rr_drive_speed = command.rr_drive_speed

        mc.drive_duration = command.drive_duration

        mc.e_stop = command.e_stop

        pub.publish(mc)

        # TODO: stuff with the command

    # Update Robot data

    rdd.data.frontLeftWheel.theta += 0.01
    rdd.data.frontLeftWheel.theta %= 1000000

