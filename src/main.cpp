// ################################################################################
// ###########################    How to use    ###################################
// ################################################################################
/*
No start-up commands necessary.
When started, it tries to open g_captureDeviceID with g_width and g_height.
It looks after the given checkerboard pattern CHECKERBOARD with the given square numbers and sizes g_squareSizeInMM.
A window 'Last seen image' will open and display the most recent image captured.
If the checkerboard corners were found, they are visible in the image aswell.
Check the quality of the match by comparing the distance between the drawn corners and the real ones.
In case the result was not good enough, it can be discarded by hitting the 'x'-key.
Otherwise hit any other button or close the shown window "Last seen image" and the app gets the next frame and tries to find the pattern again.
As long as it has not find g_minimumNumberofImagesForCalibration, it will not calibrate.
As soon as it as g_minimumNumberofImagesForCalibration reached, it tries to calibrate.
With g_maxIterationForSubPixel = 30 and g_epsilonForSubPixel = 0.01 it takes for
g_minimumNumberofImagesForCalibration = 50 about 10minutes to calculate with an Intel i9-9880H.
After the calibration, take another image of the same pattern and you will get the
cameraMatrix,
DistortionParameters
and Translation and Rotation relative between camera and pattern (in console).
At this stage the coordinate axes of the pattern can be drawn to an image by hitting the 'a'-key while the window 'Last seen image' is active. To proceed any key has to be pressed.

Usually g_mode = 0 should do fine, but check the section below "Variables to set" if not.

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

#include <rs.hpp>

//using namespace cv;
#include <stdio.h>

//#include <opencv2/aruco.hpp>
#include <string>
#include <map>
#include <algorithm>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>
#include "tinyxml2.h"

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
int g_width = 1920;
int g_height = 1080;


/*
	The index of the device to use (necessary for VideoInput and OpenCV)
	default is 0
*/
int g_captureDeviceID = 0;


/*
	Set the size of the squares on the chessboard pattern
*/
float squareSizeInMM = 35.7f;


/*
	Defining the dimensions of checkerboard (if you count the squares on the chessboard take on less. A pattern with 7 to 10 squares is a 6 to 9
*/
int CHECKERBOARD[2]{ 7,10 };


/*
	Set a capture mode:
	case 0: // Use videoInputLib
	case 1: // Use OpenCV's VideoCapture (does not run with every camera - for example not with the AV2GO USB Composite Video Grabber
	case 2: // Use OpenCV's imread() function in order to load a bunch of images from disk
	case 3: // Use RealSense Lib
	case 4: // Use Azure Kinect Sensor SDK (not implemented yet)
*/
int g_mode = 3;


/*
	Set a path to a bunch of files on disk -> used by OpenCV imgread()
	default is 0
*/
std::string g_calibrationImagePath = "C:/devel/CameraCalibrator/build/bin/CameraID1-weitwinkelMouthCam/*.jpg";


/*
	regards OpenCV:
	Criteria for termination of the iterative process of corner refinement in feature detection.
	That is, the process of corner position refinement stops either after g_maxIterationForSubPixel
	iterations or when the corner position moves by less than g_epsilonForSubPixel on some iteration.
	Have a look at https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#cornersubpix
	Default is 30 and 0.01
*/
int g_maxIterationForSubPixel = 20; //default 30 (High Quality, good pattern with big squares, good camera, takes much time for calculation)
int g_epsilonForSubPixel = 0.025; // default 0.01 (High Quality, good pattern with big squares, good camera, takes much time for calculation 
								 // faster calculation 0.03 )

int g_minimumNumberofImagesForCalibration = 50; //default = 50;

bool g_readCameraParamsFileAtStartUp = false;


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
	rs2::context rs2ctx;
	rs2::pipeline rsPipe(rs2ctx);
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
		cv::glob(g_calibrationImagePath, fn, false);
		break;
	case 3: // Use RealSense Lib (not implemented yet)
	{
		unsigned int i = 0;
		for (rs2::device&& device : rs2ctx.query_devices()) {
			if (i != g_captureDeviceID) continue;
			const char* serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			rs2::config rs2cfg;
			rs2cfg.enable_stream(RS2_STREAM_COLOR, -1, g_width, g_height, rs2_format::RS2_FORMAT_BGR8, 30);
			rs2cfg.enable_device(serial);
			rsPipe.start(rs2cfg);
			std::cout << "RealSense Device: " << serial << " started." << std::endl;
			break;
		}
	}
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
	cv::Mat R, T;
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F);
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F);
	
	//Read CameraParameters.xml and set cameraMatrix and DistCoeffs
	if (g_readCameraParamsFileAtStartUp)
	{
		std::vector<double> xmlDataCameraMatrix;
		std::vector<double> xmlDataDissCoeff;
		tinyxml2::XMLDocument xmlDoc;
		tinyxml2::XMLError eResult = xmlDoc.LoadFile("CameraParameters.xml");
		//tinyxml2::XMLCheckResult(eResult);
		tinyxml2::XMLNode* pRoot = xmlDoc.FirstChild();
		if (pRoot == nullptr) return tinyxml2::XML_ERROR_FILE_READ_ERROR;
		tinyxml2::XMLNode* cameraMatrixNode = pRoot->FirstChildElement("camera_matrix");
		if (cameraMatrixNode == nullptr) return tinyxml2::XML_ERROR_FILE_READ_ERROR;

		double e00;
		eResult = cameraMatrixNode->FirstChildElement("e00")->QueryDoubleText(&e00);
		xmlDataCameraMatrix.push_back(e00);
		
		double e01;
		eResult = cameraMatrixNode->FirstChildElement("e01")->QueryDoubleText(&e01);
		xmlDataCameraMatrix.push_back(e01);

		double e02;
		eResult = cameraMatrixNode->FirstChildElement("e02")->QueryDoubleText(&e02);
		xmlDataCameraMatrix.push_back(e02);


		double e10;
		eResult = cameraMatrixNode->FirstChildElement("e10")->QueryDoubleText(&e10);
		xmlDataCameraMatrix.push_back(e10);

		double e11;
		eResult = cameraMatrixNode->FirstChildElement("e11")->QueryDoubleText(&e11);
		xmlDataCameraMatrix.push_back(e11);

		double e12;
		eResult = cameraMatrixNode->FirstChildElement("e12")->QueryDoubleText(&e12);
		xmlDataCameraMatrix.push_back(e12);


		double e20;
		eResult = cameraMatrixNode->FirstChildElement("e20")->QueryDoubleText(&e20);
		xmlDataCameraMatrix.push_back(e20);

		double e21;
		eResult = cameraMatrixNode->FirstChildElement("e21")->QueryDoubleText(&e21);
		xmlDataCameraMatrix.push_back(e21);

		double e22;
		eResult = cameraMatrixNode->FirstChildElement("e22")->QueryDoubleText(&e22);
		xmlDataCameraMatrix.push_back(e22);
		cameraMatrix.at<double>(0, 0) = e00;
		cameraMatrix.at<double>(0, 1) = e01;
		cameraMatrix.at<double>(0, 2) = e02;
		cameraMatrix.at<double>(1, 0) = e10;
		cameraMatrix.at<double>(1, 1) = e11;
		cameraMatrix.at<double>(1, 2) = e12;
		cameraMatrix.at<double>(2, 0) = e20;
		cameraMatrix.at<double>(2, 1) = e21;
		cameraMatrix.at<double>(2, 2) = e22;

		//memcpy(cameraMatrix.data, xmlDataCameraMatrix.data(), xmlDataCameraMatrix.size() * sizeof(double));


		//Distortion Coefficients
		tinyxml2::XMLNode* distCoeffNode = pRoot->FirstChildElement("dist_coeff");
		double dC0;
		eResult = distCoeffNode->FirstChildElement("dC0")->QueryDoubleText(&dC0);
		xmlDataDissCoeff.push_back(dC0);

		double dC1;
		eResult = distCoeffNode->FirstChildElement("dC1")->QueryDoubleText(&dC1);
		xmlDataDissCoeff.push_back(dC1);

		double dC2;
		eResult = distCoeffNode->FirstChildElement("dC2")->QueryDoubleText(&dC2);
		xmlDataDissCoeff.push_back(dC2);

		double dC3;
		eResult = distCoeffNode->FirstChildElement("dC3")->QueryDoubleText(&dC3);
		xmlDataDissCoeff.push_back(dC3);

		double dC4;
		eResult = distCoeffNode->FirstChildElement("dC4")->QueryDoubleText(&dC4);
		xmlDataDissCoeff.push_back(dC4);

			
		memcpy(distCoeffs.data, xmlDataDissCoeff.data(), xmlDataDissCoeff.size() * sizeof(double));
	}




	//Main loop
	while (true)//cv::waitKey(1))
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
		case 3: // Use RealSense Lib
		{
			rs2::frameset frames = rsPipe.wait_for_frames();
			rs2::video_frame colorFrame = frames.get_color_frame();
			inputImage = cv::Mat(cv::Size(g_width, g_height), CV_8UC3, (void*) colorFrame.get_data(), cv::Mat::AUTO_STEP);
		}
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
		cv::Mat inputImageBackup = inputImage.clone();
		if (success)
		{
			std::cout << "Saw chessboard pattern - processing" << std::endl;
			cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, g_maxIterationForSubPixel, g_epsilonForSubPixel);

			// refining pixel coordinates for given 2d points.
			cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			// Displaying the detected corner points on the checker board
			cv::drawChessboardCorners(inputImage, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
		}

		// Update the window with new data
		imshow("Last seen image", inputImage);
		char key = cv::waitKey();
		if (key == 'x') {
			std::cout << "Image skipped" << std::endl;
			continue;
		}

		if (success) {
			objpoints.push_back(objp);
			imgpoints.push_back(corner_pts);

			seenCheckerImages++;
		}

		std::cout << "seenCheckerImages : " << seenCheckerImages << std::endl;

		//Calibration
		if (success && seenCheckerImages == g_minimumNumberofImagesForCalibration)
		{
			std::cout << "Vor calibrateCamera()" << std::endl;
			cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
			//Does not work -> It wil crashes
			//cv::fisheye::calibrate(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
			std::cout << "Nach calibrateCamera()" << std::endl;
			std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
			std::cout << "distCoeffs : " << distCoeffs << std::endl;
			std::cout << "Rotation vector : " << R << std::endl;
			std::cout << "Translation vector : " << T << std::endl;
			std::cout << "Raus_______________________________" << std::endl;




			//Init XML File for saving cam params
			tinyxml2::XMLDocument doc;
			tinyxml2::XMLElement* highestNode = doc.NewElement("camera_params");
			tinyxml2::XMLElement* cameraMatrixNode = doc.NewElement("camera_matrix");
			{
				//First row
				tinyxml2::XMLElement* e00 = doc.NewElement("e00");
				e00->SetText(cameraMatrix.at<double>(0, 0));
				cameraMatrixNode->InsertEndChild(e00);

				tinyxml2::XMLElement* e01 = doc.NewElement("e01");
				e01->SetText(cameraMatrix.at<double>(0, 1));
				cameraMatrixNode->InsertEndChild(e01);

				tinyxml2::XMLElement* e02 = doc.NewElement("e02");
				e02->SetText(cameraMatrix.at<double>(0, 2));
				cameraMatrixNode->InsertEndChild(e02);

				//Second row
				tinyxml2::XMLElement* e10 = doc.NewElement("e10");
				e10->SetText(cameraMatrix.at<double>(1, 0));
				cameraMatrixNode->InsertEndChild(e10);

				tinyxml2::XMLElement* e11 = doc.NewElement("e11");
				e11->SetText(cameraMatrix.at<double>(1, 1));
				cameraMatrixNode->InsertEndChild(e11);

				tinyxml2::XMLElement* e12 = doc.NewElement("e12");
				e12->SetText(cameraMatrix.at<double>(1, 2));
				cameraMatrixNode->InsertEndChild(e12);
				
				//Third row
				tinyxml2::XMLElement* e20 = doc.NewElement("e20");
				e20->SetText(cameraMatrix.at<double>(2, 0));
				cameraMatrixNode->InsertEndChild(e20);

				tinyxml2::XMLElement* e21 = doc.NewElement("e21");
				e21->SetText(cameraMatrix.at<double>(2, 1));
				cameraMatrixNode->InsertEndChild(e21);

				tinyxml2::XMLElement* e22 = doc.NewElement("e22");
				e22->SetText(cameraMatrix.at<double>(2, 2));
				cameraMatrixNode->InsertEndChild(e22);

			}
			highestNode->InsertEndChild(cameraMatrixNode);
			tinyxml2::XMLElement* distCoeff = doc.NewElement("dist_coeff");
			{
				tinyxml2::XMLElement* dC0 = doc.NewElement("dC0");
				dC0->SetText(distCoeffs.at<double>(0));
				distCoeff->InsertEndChild(dC0);

				tinyxml2::XMLElement* dC1 = doc.NewElement("dC1");
				dC1->SetText(distCoeffs.at<double>(1));
				distCoeff->InsertEndChild(dC1);

				tinyxml2::XMLElement* dC2 = doc.NewElement("dC2");
				dC2->SetText(distCoeffs.at<double>(2));
				distCoeff->InsertEndChild(dC2);

				tinyxml2::XMLElement* dC3 = doc.NewElement("dC3");
				dC3->SetText(distCoeffs.at<double>(3));
				distCoeff->InsertEndChild(dC3);

				tinyxml2::XMLElement* dC4 = doc.NewElement("dC4");
				dC4->SetText(distCoeffs.at<double>(4));
				distCoeff->InsertEndChild(dC4);
			}
			highestNode->InsertEndChild(distCoeff);
			doc.InsertEndChild(highestNode);
			doc.SaveFile("CameraParameters.xml");
		}

		if (success && seenCheckerImages > g_minimumNumberofImagesForCalibration)
		{
			std::cout << "Vor solvePnP()" << std::endl;
			int imagesTaken = objpoints.size();
			cv::solvePnPRansac(objpoints.at(imagesTaken - 1), imgpoints.at(imagesTaken - 1), cameraMatrix, distCoeffs, R, T);
			//cv::solvePnP(objpoints.at(imagesTaken - 1), imgpoints.at(imagesTaken - 1), cameraMatrix, distCoeffs, R, T);
			std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
			std::cout << "distCoeffs : " << distCoeffs << std::endl;
			std::cout << "Rotation vector : " << R << std::endl;
			std::cout << "Translation vector : " << T << std::endl;
			std::cout << "Raus_______________________________" << std::endl;
			//objpoints.clear();
			//imgpoints.clear();
			//Mat temp = inputImage.clone();
			//cv::Mat temp = inputImage.clone();
			//undistort(temp, inputImage, cameraMatrix, distCoeffs);
			cv::Mat undistortedImage;
			undistort(inputImage, undistortedImage, cameraMatrix, distCoeffs);
			imshow("Undistored", undistortedImage);
			if (key == 'a') {
				cv::drawFrameAxes(inputImageBackup, cameraMatrix, distCoeffs, R, T, 100.f);
				std::cout << "DrawFrameAxis: press key to continue" << std::endl;
				imshow("Last seen image", inputImageBackup);
				cv::waitKey();
			}
		}

		if (success && g_readCameraParamsFileAtStartUp)
		{
			int imagesTaken = objpoints.size();
			cv::solvePnP(objpoints.at(imagesTaken - 1), imgpoints.at(imagesTaken - 1), cameraMatrix, distCoeffs, R, T);
			std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
			std::cout << "distCoeffs : " << distCoeffs << std::endl;
			std::cout << "Rotation vector : " << R << std::endl;
			std::cout << "Translation vector : " << T << std::endl;
		}

		//cv::waitKeyEx(0);
	}
	rsPipe.stop();
	cv::waitKey(0);
	return EXIT_SUCCESS;
}
