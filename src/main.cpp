// ################################################################################
// ###########################    How to use    ###################################
// ################################################################################
/*
Not start up commands necessary.
When started, it tries to open g_captureDeviceID with g_width and g_height.
It looks after the given checkerboard pattern CHECKERBOARD with the given square numbers and sizes g_squareSizeInMM.
Close the shown window "Last seen image" and the app gets the next frame and tries to find the pattern.
As long as it has not find g_minimumNumberofImagesForCalibration, it will not calibrate.
As soon as it as g_minimumNumberofImagesForCalibration reached, it tries to calibrate.
With g_maxIterationForSubPixel = 30 and g_epsilonForSubPixel = 0.01 it takes for
g_minimumNumberofImagesForCalibration = 50 about 10minutes to calculate in Intel i9. 
After this, take another photo of the same pattern and you will get the 
cameraMatrix,
DistortionParameters
and Translation and Rotion relative between camera an pattern.

Usually g_mode = 0 should do fine, but check the section "Variables to set" if not below.

Happy calibrating :)

*/
// ################################################################################
// ################################################################################
// ################################################################################


//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
// BETTER CAMERA CALIB
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//using namespace cv;
#include <stdio.h>

//#include <opencv2/aruco.hpp>
#include <string>
#include <map>
#include <algorithm>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>
//#include "tinyxml2.h"

#include <videoInput.h>
videoInput VI;
unsigned char* viFrame;


// ################################################################################
// ########################### Variables to set ###################################
// ################################################################################
/*
	Width and height of the capture size
	Does not work for OpenCV
*/
int g_width = 640;
int g_height = 480;


/*
	The index of the device to use (necessary for VideoInput and OpenCV)
	default is 0
*/
int g_captureDeviceID = 0;


/*
	Set the size of the squares on the chessboard pattern
*/
float squareSizeInMM = 10.0f;


/*
	Defining the dimensions of checkerboard (if you count the squares on the chessboard take on less. A pattern with 7 to 10 squares is a 6 to 9
*/
int CHECKERBOARD[2]{ 6,9 };


/* 
	Set a capture mode:
	case 0: // Use videoInputLib
	case 1: // Use OpenCV's VideoCapture (does not run with every camera - for example not with the AV2GO USB Composite Video Grabber
	case 2: // Use OpenCV's imread() function in order to load a bunch of images from disk
	case 3: // Use RealSense Lib (not implemented yet)
	case 4: // Use Azure Kinect Sensor SDK (not implemented yet)
*/
int g_mode = 0;


/* 
	Set a path to a bunch of files on disk -> used by OpenCV imgread()
	default is 0
*/
std::string g_path = "C:/devel/CameraCalibrator/build/bin/CameraID1-weitwinkelMouthCam/*.jpg";


/*
	regards OpenCV:
	Criteria for termination of the iterative process of corner refinement in feature detection.
	That is, the process of corner position refinement stops either after g_maxIterationForSubPixel
	iterations or when the corner position moves by less than g_epsilonForSubPixel on some iteration.
	Have a look at https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#cornersubpix
	Default is 30 and 0.01
*/
int g_maxIterationForSubPixel = 30;
int g_epsilonForSubPixel = 0.01;





int g_minimumNumberofImagesForCalibration = 50;



// ################################################################################
// ################################################################################
// ################################################################################



// ################################################################################
// ###########################    Dev notes     ###################################
// ################################################################################
/*
	Experimetal State
	For RealSense implementation in the future, look at the file 
	"oldRealSenseCalibratorFromStudentProject.txt"


*/
// ################################################################################
// ################################################################################
// ################################################################################


/*
Main Funktion
*/
int main(int argc, char* argv[])
{
	std::vector<std::string> fn;
	cv::VideoCapture cap;
	cv::Mat inputImage;
	switch (g_mode) {
	case 0: // Use videoInputLib
		VI.setUseCallback(true);
		VI.setupDevice(g_captureDeviceID, g_width, g_height, VI_COMPOSITE);
		viFrame = new unsigned char[VI.getSize(g_captureDeviceID)];
		inputImage = cv::Mat(cv::Size(g_width, g_height), CV_8UC3, (void*)viFrame, cv::Mat::AUTO_STEP);
		break;
	case 1: // Use OpenCV's VideoCapture (does not run with every camera - for example not with the AV2GO USB Composite Video Grabber
		if (!cap.open(g_captureDeviceID))
		{
			std::cout << "Can't open OpenCV CaptureDevice" << std::endl;
		}
		cap.grab();
		cap.set(cv::CAP_PROP_FRAME_WIDTH, g_width); //Does not work?
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, g_height);
		break;
	case 2: // Use OpenCV's imread() function in order to load a bunch of images from disk
		cv::glob(g_path, fn, false);
		break;
	case 3: // Use RealSense Lib (not implemented yet)
		break;
	case 4: // Use Azure Kinect Sensor SDK (not implemented yet)
		break;
	default:
		break;
	}

	//Set up of calibration 
	// Creating vector to store vectors of 3D points for each checkerboard image
	std::vector<std::vector<cv::Point3f> > objpoints;

	// Creating vector to store vectors of 2D points for each checkerboard image
	std::vector<std::vector<cv::Point2f> > imgpoints;

	// Defining the world coordinates for 3D points
	std::vector<cv::Point3f> objp;
	for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
	{
		for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
			objp.push_back(cv::Point3f(j * squareSizeInMM, i * squareSizeInMM, 0.0f));
	}

	int seenCheckerImages = 0;
	cv::Mat cameraMatrix, distCoeffs, R, T;


	//Main loop
	while (cv::waitKey(1))
	{
		switch (g_mode) {
		case 0: // Use videoInputLib
			if (VI.isFrameNew(g_captureDeviceID))
			{
				//we get the pixels by passing in out buffer which gets filled
				VI.getPixels(g_captureDeviceID, viFrame, true);
			}
			cv::cvtColor(inputImage, inputImage, cv::COLOR_RGB2BGR);
			cv::flip(inputImage, inputImage, 0);
			break;
		case 1: // Use OpenCV's VideoCapture (does not run with every camera - for example not with the AV2GO USB Composite Video Grabber
			cap >> inputImage;
			break;
		case 2:// Use OpenCV's imread() function in order to load a bunch of images from disk
		{
			int readFileCounter = 0;
			if (fn.size() > readFileCounter)
			{
				std::cout << "imread file path: " << fn.at(readFileCounter) << std::endl;
				inputImage = cv::imread(fn.at(readFileCounter));
				readFileCounter++;
			}
		}
			break;
		case 3: // Use RealSense Lib (not implemented yet)
			break;
		case 4: // Use Azure Kinect Sensor SDK (not implemented yet)
			break;
		default:
			break;
		}

		
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

		std::vector<cv::Point2f> corner_pts;
		bool success;

		cv::Mat gray;
		cv::cvtColor(inputImage, gray, cv::COLOR_RGB2GRAY);

		// Finding checker board corners
		// If desired number of corners are found in the image then success = true  
		success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | 
																											cv::CALIB_CB_NORMALIZE_IMAGE | 
																											cv::CALIB_CB_FILTER_QUADS);
		//success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts);
		
		/*
		 * If desired number of corner are detected,
		 * we refine the pixel coordinates and display
		 * them on the images of checker board
		*/
		if (success)
		{
			std::cout << "Saw chessboard pattern - processing" << std::endl;
			cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, g_maxIterationForSubPixel, g_epsilonForSubPixel);

			// refining pixel coordinates for given 2d points.
			cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			// Displaying the detected corner points on the checker board
			cv::drawChessboardCorners(inputImage, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

			objpoints.push_back(objp);
			imgpoints.push_back(corner_pts);

			seenCheckerImages++;
		}


		// Update the window with new data
		imshow("Last seen image", inputImage);
		std::cout << "seenCheckerImages : " << seenCheckerImages << std::endl;

		//Calibration
		if (seenCheckerImages == g_minimumNumberofImagesForCalibration)
		{
			std::cout << "Vor calibrateCamera()" << std::endl;
			cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
			std::cout << "Nach calibrateCamera()" << std::endl;
		}

		if (seenCheckerImages > g_minimumNumberofImagesForCalibration)
		{
			std::cout << "Vor solvePnP()" << std::endl;
			int imagesTaken = objpoints.size();
			cv::solvePnP(objpoints.at(imagesTaken - 1), imgpoints.at(imagesTaken - 1), cameraMatrix, distCoeffs, R, T);
			std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
			std::cout << "distCoeffs : " << distCoeffs << std::endl;
			std::cout << "Rotation vector : " << R << std::endl;
			std::cout << "Translation vector : " << T << std::endl;
			std::cout << "Raus_______________________________" << std::endl;
			//objpoints.clear();
			//imgpoints.clear();
			//Mat temp = inputImage.clone();
			cv::Mat temp = inputImage.clone();
			undistort(temp, inputImage, cameraMatrix, distCoeffs);
			imshow("Undistored", inputImage);
		}
		cv::waitKeyEx(0);
	}
	cv::waitKey(0);
	return EXIT_SUCCESS;
}
