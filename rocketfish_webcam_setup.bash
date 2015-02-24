#!/bin/bash
# This script sets up the usb_cam parameters such that the module
# works with the Rocketfish AF webcam.
rosparam set usb_cam/framerate 15
rosparam set usb_cam/image_height 600
rosparam set usb_cam/image_width 800
rosparam set usb_cam/pixel_format "mjpeg"
rosparam set usb_cam/io_method "mmap"
