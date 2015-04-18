source /home/pi/ROSta-Bot/devel/setup.bash

boardConfig="/home/pi/ROSta-Bot/src/position_sensoring/985_meters.yml"
cameraConfig="/home/pi/ROSta-Bot/src/position_sensoring/pseye_calibration.yml"

rosrun position_sensoring position_sensoring_node $boardConfig $cameraConfig
