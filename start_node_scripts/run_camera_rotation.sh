source /home/pi/ROSta-Bot/devel/setup.bash

boardConfig="/home/pi/Pictures/two_marker_board.yml"
cameraConfig="/home/pi/Pictures/rocketfish_calibration.yml"

rosrun position_sensoring position_sensoring_node $boardConfig $cameraConfig
