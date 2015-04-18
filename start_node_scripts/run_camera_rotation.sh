source /home/pi/ROSta-Bot/devel/setup.bash

largeBoardConfig="/home/pi/Pictures/985_meters.yml"
smallBoardConfig="/home/pi/Pictures/smallMarker.yml"
cameraConfig="/home/pi/Pictures/rocketfish_calibration.yml"

rosrun position_sensoring position_sensoring_node $largeBoardConfig $smallBoardConfig $cameraConfig
