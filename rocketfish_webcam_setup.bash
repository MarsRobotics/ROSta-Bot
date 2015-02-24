#!/bin/bash
rosparam set usb_cam/framerate 15
rosparam set usb_cam/image_height 600
rosparam set usb_cam/image_width 800
rosparam set usb_cam/pixel_format "mjpeg"
rosparam set usb_cam/io_method "mmap"
