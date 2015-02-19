#!/usr/bin/env python

import os
import sys
import rospy
import rospkg
import roslib
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QLabel, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QCheckBox, QWidget, QToolBar, QLineEdit, QPushButton, QGraphicsView, QApplication
import thread

import Tkinter

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

message = 'hi';

class View(QGraphicsView):
	def __init__(self, parent=None):
       	 	super(View, self).__init__()

class Gui(QWidget):

	def __init__(self, context):
		super(Gui, self).__init__()
	        rp = rospkg.RosPack()
		self._widget = QWidget()
		ui_file = os.path.join(rp.get_path('manual_interface'), 'resource', 'gui.ui')
       	 	#loadUi(ui_file, self._widget, {'View': View})
		self.setObjectName("Plugin")
		self._widget.setObjectName("Gui")
		context.add_widget(self._widget)
        	self._label = QLabel("hi")
		self._label.setObjectName("Label")
		context.add_widget(self._label)


class listenerObj(object):
	def __init__(self, window):
		self.window = window
	
	# invoked when subscriber gets a new message
	def guiCallback(self, data):
		# print to console
		rospy.loginfo("Data: %s", data.data)
		self.window.setText("Yay: " + data.data)
		# display message in gui
	
	def listener(self):
		# init ros system
		rospy.init_node('gui', anonymous=True)

		# testing
		rospy.loginfo("wee")

		# create a subscriber that listens to "gui" topics
		rospy.Subscriber("gui", String, self.guiCallback);




if __name__ == '__main__':
	window = QApplication(sys.argv)
	myLabel = QLabel("Say 'Hello world!'")

	# The rest is known already...
	myLabel.show()

	listenerObj(myLabel).listener()
	window.exec_()
	rospy.spin()


# Good tutorials to reference if stuck

#http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

#http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#http://wiki.ros.org/rqt/Tutorials/Using%20.ui%20file%20in%20rqt%20plugin

# rqt --standalone manual_interface_node

