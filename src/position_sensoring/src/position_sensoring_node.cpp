#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
//TODO viki needs to be changed when using Rpi
#include </home/viki/ROSta-Bot/devel/include/position_sensoring/position.h>
#include </home/viki/ROSta-Bot/devel/include/position_sensoring/marker_visible.h>

using namespace cv;
using namespace aruco;
using namespace position_sensoring;

int currentCameraAngle = -1;
int targetAngle = -1;

// Copied from ArUco library
void getObjectAndImagePoints(Board &B, vector<cv::Point3f> &objPoints,vector<cv::Point2f> &imagePoints) {
    //composes the matrices
    int nPoints=B.size()*4;

    int cIdx=0;
    //for (size_t i=0;i<B.size();i++)
	const aruco::MarkerInfo  & mInfo=B.conf.getMarkerInfo(B[0].id);
	for (int j=0;j<4;j++,cIdx++) {
		imagePoints.push_back(B[0][j]);
		objPoints.push_back(  mInfo[j]);
	}
    //}
}

/**
 * current_camera_angle callback
 */
void current_camera_angle_callback(const std_msgs::Int32::ConstPtr& message) {
	// upcate current_camera_angle
	currentCameraAngle = message->data;
}

int main(int argc,char **argv) {
	try {
		if (argc != 3) {
			cerr << "Usage: boardConfig.yml  camera_config.yml " << endl;
			return -1;
		}

		ros::init(argc, argv, "position_sensor");
		ros::NodeHandle n;

		// publish to topic "target_distance", hold 1000 of these message in the buffer before discarding
		ros::Publisher positionPub = n.advertise<position_sensoring::position>("range_data", 1000);
		position_sensoring::position positionMsg;

		// publish to the topic "marker_visible" hold 1000 of these messages in the buffer before discarding
		ros::Publisher markerPub = n.advertise<position_sensoring::marker_visible>("is_marker_visible", 1000);
		position_sensoring::marker_visible markerMsg;

		// publish new target angle to Arduino
		ros::Publisher updateTargetAngle = n.advertise<std_msgs::Int32>("target_camera_angle", 1000);

		// listen to current angle of camera (relative to motor)
		ros::Subscriber updateCurrentCameraAngle = n.subscribe("current_camera_angle", 1, current_camera_angle_callback);


		// Variables to hold our boards,and configurations.
		BoardConfiguration boardConfig;
		BoardDetector boardDetector;
		CameraParameters cameraParams;
		
		// Read the large board configuration file
		boardConfig.readFromFile(argv[1]);

		//Get the camera configurations
		cameraParams.readFromXMLFile(argv[2]);

		VideoCapture videoCapture;
		cv::Mat inputImage;

		//Open the camera
		videoCapture.open(0);

		//make sure camera is open
		if (!videoCapture.isOpened()) {
			cerr << "Could not open the video" << endl;
			return -1;
		}

		Vector<Marker> markers;

		//read an image from camera
		videoCapture >> inputImage;

		boardDetector.setParams(boardConfig, cameraParams);

		while (videoCapture.grab()) {
			//retrieve an image from the camera
			videoCapture.retrieve(inputImage);

			// Detect the images
			boardDetector.detect(inputImage);
			markers = boardDetector.getDetectedMarkers();
			Board board  = boardDetector.getDetectedBoard();


			// Determine pose and position based on the seen marker
			if(markers.size() > 0) {
				vector <cv::Point3f> objPoints;
				vector <cv::Point2f> imgPoints;

				// get "3d" points of the object in real space, get "2d" points of the object in a flat image
				getObjectAndImagePoints(board, objPoints, imgPoints);

				// solve for transition matrix and store in rvec
				// solve for rotation matrix and store in tvec
				cv::Mat rvec, tvec;
				solvePnP(objPoints, imgPoints, cameraParams.CameraMatrix, cameraParams.Distorsion, rvec, tvec);

				// determine the position of the "camera" --> which in our case, will be the robot as well
				cv::Mat R;
				cv::Rodrigues(rvec, R);
				cv::Mat cameraRotationVector;
				cv::Rodrigues(R.t(), cameraRotationVector);
				cv::Mat cameraTranslationVector = -R.t() * tvec;

				// converting data from meters to centimeters
				positionMsg.xDistance = cameraTranslationVector.at<double>(0) * 100.0;
				positionMsg.yDistance = cameraTranslationVector.at<double>(1) * 100.0;
				positionMsg.zDistance = cameraTranslationVector.at<double>(2) * 100.0;

				positionMsg.xPose = cameraRotationVector.at<double>(0) * 100.0;
				positionMsg.yPose = cameraRotationVector.at<double>(1) * 100.0;
				positionMsg.zPose = cameraRotationVector.at<double>(2) * 100.0;

 				positionPub.publish(positionMsg);

				// if the beacon is about to not be visible, rotate the camera
				// starting with a small threshold first for testing purposes
				if(positionMsg.yPose > 3 || positionMsg.yPose < -3){
					if(positionMsg.yPose > 3 ){
						targetAngle = currentCameraAngle - 3;
					}
					else {
						targetAngle = currentCameraAngle + 3;	
					}
					std_msgs::Int32 theta;
					theta.data = targetAngle;
					updateTargetAngle.publish(theta);
				}		
			}
				
			// not sure where this really belongs...
  			ros::spinOnce();
		}

		return 0;
	} // end try
	catch (std::exception &ex) {
		cout << "Position Sensoring Exception :" << ex.what() << endl;
	} // end catch

} // end main


