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

//The size of the marker in meters
const float MARKER_SIZE = .16;
const float DISTANCE_CONSTANT = 7183.16666666666666;

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
Mat rvec, tvec;				// for storing translation and rotation matrices

int boardHeight = 500;
int boardWidth = 500;
Vector<Marker> markers;





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

int main(int argc,char **argv)
{
    try
    {
        //if (argc!=3) {
        //    cerr<<"Usage: boardConfig.yml camera_config.yml "<<endl;
        //    return -1;
        //}

        //Get the camera config
       	//cameraConfigFile = argv[2];
       	cameraParams.readFromXMLFile("/home/viki/ROSta-Bot/src/position_sensoring/src/cam_param.yml");

        //Read the board
        //boardConfigFile = argv[1];
        boardConfig.readFromFile("/home/viki/ROSta-Bot/src/position_sensoring/src/single_marker.yml");

        //Open the camera

        videoCapture.open(0);
//        videoCapture.open("/home/mars/Dropbox/MarsFiles/marker.avi");

        //make sure camera is open
        if (!videoCapture.isOpened()) {
            cerr<<"Could not open the video"<<endl;
            return -1;
        }

        //read an image from camera
        videoCapture >> inputImage;
	//cameraParams.resize(inputImage);

        //create the gui
        cv::namedWindow("output video", CV_WINDOW_AUTOSIZE);

        boardDetector.setParams(boardConfig, cameraParams);

	vector<Point3d> boardPoints;


	//generate vectors for the points on the chessboard
	for (int i=0; i<boardWidth; i++)
	{
		for (int j=0; j<boardHeight; j++)
		{
			boardPoints.push_back( Point3d( double(i), double(j), 0.0) );
		}
	}



        while ( inputKey != 27 && videoCapture.grab()) {

            //retrieve an image from the camera
            videoCapture.retrieve(inputImage);
            inputImage.copyTo(inputImageCopy);


            boardDetector.detect(inputImage);
            markers = boardDetector.getDetectedMarkers();
            for (unsigned int i = 0; i < markers.size(); i++) {
                Marker m = markers[i];
		/*
                float camData[] = {1.3089596458327883e+03, 0.0, 3.1716402612677911e+02, 0.0, 1.3249652390726415e+03, 2.3359932814285278e+02, 0., 0., 1.0};
                Mat cam = Mat(3, 3, CV_32F, camData);
                float distortionData[] = {1.8869609562810794e+00, 4.2431486029577115e+01,
                        -7.9002380674102313e-02, -1.0615309141897006e-02,
                        -1.2940788383601671e+03};
                Mat distortion = Mat(1, 5, CV_32F, distortionData);
                Size size = Size(640, 480);

                CameraParameters cp = CameraParameters(cam, distortion, size);

//                This isn't needed I guess
//                m.calculateExtrinsics(MARKER_SIZE, cp);
*/ 
                Point topLeft = m[0];
                Point topRight = m[1];
                Point bottomRight = m[2];
                Point bottomLeft = m[3];
             
		vector<Point2f> corners;

                //Display data to the meatbag humans

                for (int i=0;i<4;i++){
                    cout<<"("<<m[i].x<< ","<<m[i].y<<") ";
		    Point2f a(m[i].x, m[i].y);
		    corners.push_back(a);
		}
                cout << endl;
		


    		cv::Mat CamMatrix=cv::Mat::eye(3,3,CV_32F);
    		CamMatrix.at<float>(0,0)=500;
    		CamMatrix.at<float>(1,1)=500;
    		CamMatrix.at<float>(0,2)=inputImageCopy.size().width/2;
    		CamMatrix.at<float>(1,2)=inputImageCopy.size().height/2;
    		cameraParams.setParams(CamMatrix,cv::Mat::zeros(1,5,CV_32F) ,inputImageCopy.size());

             	vector<cv::Point3f> objPoints;
		vector<cv::Point2f> imgPoints;
                getObjectAndImagePoints(boardDetector.getDetectedBoard(),objPoints,imgPoints);


		solvePnP(objPoints, imgPoints, cameraParams.CameraMatrix, cameraParams.Distorsion, rvec, tvec);

		cv::Mat R;
		cv::Rodrigues(rvec, R);
		cv::Mat cameraRotationVector;
		cv::Rodrigues(R.t(), cameraRotationVector);

		cv::Mat cameraTranslationVector = -R.t()*tvec;

		cout << "Camera position " << cameraTranslationVector.at<double>(0) << ", " << cameraTranslationVector.at<double>(1) << ", " << cameraTranslationVector.at<double>(2) << endl;
 
cout << "Camera pose " << cameraRotationVector.at<double>(0) << ", " << cameraRotationVector.at<double>(1) << ", " << cameraRotationVector.at<double>(2) << endl;
 


                m.draw(inputImageCopy, COLOR, LINE_WIDTH);

            }

            cv::imshow("output video",inputImageCopy);


            //Check if the stop button has been pressed
            inputKey = cv::waitKey(WAIT_TIME);
        }

        return 0;

    } catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}


