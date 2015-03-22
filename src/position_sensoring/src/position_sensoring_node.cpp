#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;
using namespace aruco;

//Width of line to highlight marker with
const int LINE_WIDTH = 1;

//Color to highlight marker lines with
const Scalar COLOR = (0, 0, 255);

//Delay before next image retrieval
const int WAIT_TIME = 2;

//Correction factor for the Z-value (distance between marker and camera).
const float Z_VALUE_CORRECTION = -1.140;

//Integer representation of which keyboard key was pressed
int inputKey = 0;

//Path to files
string boardConfigFile;
string cameraConfigFile;

VideoCapture videoCapture;
Mat inputImage,inputImageCopy;
BoardConfiguration boardConfig;
BoardDetector boardDetector;
CameraParameters cameraParams;
Mat rvec, tvec;		// for storing translation and rotation matrices

int boardHeight = 500;
int boardWidth = 500;
Vector<Marker> markers;

// Copied from ArUco library
void getObjectAndImagePoints(Board &B, vector<cv::Point3f> &objPoints,vector<cv::Point2f> &imagePoints) {
    //composes the matrices
    int nPoints=B.size()*4;

    int cIdx=0;
    for (size_t i=0;i<B.size();i++) {
        const aruco::MarkerInfo  & mInfo=B.conf.getMarkerInfo(B[i].id);
        for (int j=0;j<4;j++,cIdx++) {
            imagePoints.push_back(B[i][j]);
            objPoints.push_back(  mInfo[j]);
        }
    }
}

int main(int argc,char **argv) {
	try {
		if (argc != 3) {
			cerr << "Usage: boardConfig.yml camera_config.yml " << endl;
			return -1;
		}

		//Read the board configuration file
		boardConfigFile = argv[1];
		boardConfig.readFromFile(boardConfigFile);
		//IE: boardConfig.readFromFile("/home/viki/ROSta-Bot/src/position_sensoring/src/single_marker.yml");


		//Get the camera configurations
		cameraConfigFile = argv[2];
		cameraParams.readFromXMLFile(cameraConfigFile);
		// IE: cameraParams.readFromXMLFile("/home/viki/ROSta-Bot/src/position_sensoring/src/cam_param.yml");

		//Open the camera
		videoCapture.open(0);

		//make sure camera is open
		if (!videoCapture.isOpened()) {
			cerr << "Could not open the video" << endl;
			return -1;
		}

		//read an image from camera
		videoCapture >> inputImage;

		//create the gui
		cv::namedWindow("output video", CV_WINDOW_AUTOSIZE);

		boardDetector.setParams(boardConfig, cameraParams);

		while (inputKey != 27 && videoCapture.grab()) {
			//retrieve an image from the camera
			videoCapture.retrieve(inputImage);

			// save a copy of it somewhere, in case our calculations make changes
			inputImage.copyTo(inputImageCopy);
			boardDetector.detect(inputImage);

			// detect the markers, and for each marker, determine pose and position
			markers = boardDetector.getDetectedMarkers();
			for (unsigned int i = 0; i < markers.size(); i++) {
				Marker m = markers[i];

				//Display data to the meatbag humans
				for (int i = 0; i < 4; i++) {
					cout << "(" << m[i].x << "," << m[i].y << ") ";
					Point2f a(m[i].x, m[i].y);
				}
				cout << endl;

				// brute force way to set up the camera matrix and parameters...may need to calibrate using ArUco
				// if we do decide to use a calibrated parameter, then get ride of these lines...
				// we read from a board configuration, so probably dont need to do this. hopefully
				// cv::Mat CamMatrix = cv::Mat::eye(3, 3, CV_32F);
				//CamMatrix.at<float>(0, 0) = 500;
				//CamMatrix.at<float>(1, 1) = 500;
				//CamMatrix.at<float>(0, 2) = inputImageCopy.size().width / 2;
				//CamMatrix.at<float>(1, 2) = inputImageCopy.size().height / 2;
				//cameraParams.setParams(CamMatrix, cv::Mat::zeros(1, 5, CV_32F), inputImageCopy.size());

				vector <cv::Point3f> objPoints;
				vector <cv::Point2f> imgPoints;

				// get "3d" points of the object in real space, get "2d" points of the object in a flat image
				getObjectAndImagePoints(boardDetector.getDetectedBoard(), objPoints, imgPoints);

				// solve for transition matrix and store in rvec
				// solve for rotation matrix and store in tvec
				solvePnP(objPoints, imgPoints, cameraParams.CameraMatrix, cameraParams.Distorsion, rvec, tvec);

				// determine the position of the "camera" --> which in our case, will be the robot as well
				cv::Mat R;
				cv::Rodrigues(rvec, R);
				cv::Mat cameraRotationVector;
				cv::Rodrigues(R.t(), cameraRotationVector);
				cv::Mat cameraTranslationVector = -R.t() * tvec;

				// print the translation vectors (distance between marker and camera in x, y, z direction)
				cout << "Camera position " << cameraTranslationVector.at<double>(0) << ", " << cameraTranslationVector.at<double>(1) << ", " << cameraTranslationVector.at<double>(2) * Z_VALUE_CORRECTION << endl;

				// print the pose vectors (orientation between marker and camera in x,y,z direction -- angle of rotation for each)
				cout << "Camera pose " << cameraRotationVector.at<double>(0) << ", " << cameraRotationVector.at<double>(1) << ", " << cameraRotationVector.at<double>(2) << endl;

				m.draw(inputImageCopy, COLOR, LINE_WIDTH);
			}
			// update the frame!
			cv::imshow("output video", inputImageCopy);

			//Check if the stop (ESC) button has been pressed
			inputKey = cv::waitKey(WAIT_TIME);
		}

		return 0;
	} // end try
	catch (std::exception &ex) {
		cout << "Exception :" << ex.what() << endl;
	} // end catch

} // end main


