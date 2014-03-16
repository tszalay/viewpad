#include "stdafx.h"
#include "vpStereoCamera.h"
#include "LVUVCPublic.h"

using namespace std;
using namespace cv;

bool vpStereoCamera::init()
{
	//Prints out a list of available devices and returns num of devices found
	int numDevices = VI.listDevices();	
	
	deviceLeft = 0;  //this could be any deviceID that shows up in listDevices
	deviceRight = 1;
	
	//setup the first device - there are a number of options:
	VI.setIdealFramerate(deviceLeft, DEF_FPS);
	VI.setIdealFramerate(deviceRight, DEF_FPS);

	// attempt to set both devices
	if(!VI.setupDevice(deviceLeft, DEF_WIDTH, DEF_HEIGHT))
		return false;
	if(!VI.setupDevice(deviceRight, DEF_WIDTH, DEF_HEIGHT))
		return false;
	 
	// set our various parameters
	width 	= VI.getWidth(deviceLeft);
	height 	= VI.getHeight(deviceLeft);
	imgSize	= VI.getSize(deviceLeft);
	if (verbose)
		printf("Devices initialized at %dx%d, img=%d bytes\n",width,height,imgSize);

	VI.setVideoSettingFilterPct(deviceLeft, VI.propSaturation, 0);
	VI.setVideoSettingFilterPct(deviceRight, VI.propSaturation, 0);

	// set some filter parameters as we want them to be
	VI.setVideoSettingFilterPct(deviceLeft, VI.propSharpness, 0.3);
	VI.setVideoSettingFilterPct(deviceRight, VI.propSharpness, 0.3);

	// set zoom to zero
	VI.setLogitechPTZ(deviceLeft,0,0,100);
	VI.setLogitechPTZ(deviceRight,0,0,100);

	// and the exposure
	VI.setLogitechExposure(deviceLeft, 40.0f);
	VI.setLogitechExposure(deviceRight, 40.0f);
	VI.setVideoSettingCameraPct(deviceLeft, VI.propGain, 0.3);
	VI.setVideoSettingCameraPct(deviceRight, VI.propGain, 0.3);

	// control LED, just 'cause we can :-p
	KSPROPERTY_LP1_LED_S ledValue;
	ledValue.ulMode = LVUVC_LP1_LED_MODE_OFF;
	ledValue.ulFrequency = 1;
	VI.setLogitechSetting(deviceLeft, KSPROPERTY_LP1_LED, &ledValue, sizeof(ledValue));
	VI.setLogitechSetting(deviceRight, KSPROPERTY_LP1_LED, &ledValue, sizeof(ledValue));

	// create the internal image
	buffer = new unsigned char[imgSize];
	imgLeft = Mat(height, width, CV_8UC3);
	imgRight = Mat(height, width, CV_8UC3);
	imgGrayLeft = Mat(height, width, CV_8UC1);
	imgGrayRight = Mat(height, width, CV_8UC1);

	// attempt to load calibration data
	this->loadCalibration();

	return true;
}

bool vpStereoCamera::pollCamera()
{
	// check for a new frame, retrieve, and set OpenCV image
	if (VI.isFrameNew(deviceLeft)){
		VI.getPixels(deviceLeft, buffer, false, true);
		Mat tmp = Mat(height, width, CV_8UC3, buffer);
		tmp.copyTo(imgLeft);
		cv::cvtColor(imgLeft, imgGrayLeft, CV_BGR2GRAY);
		newDataLeft = true;
	}
	if (VI.isFrameNew(deviceRight)){
		VI.getPixels(deviceRight, buffer, false, true);
		Mat tmp = Mat(height, width, CV_8UC3, buffer);
		tmp.copyTo(imgRight);
		cv::cvtColor(imgRight, imgGrayRight, CV_BGR2GRAY);
		newDataRight = true;
	}
	// only flag if both cameras have a new image
	if (newDataLeft && newDataRight)
	{
		newDataLeft = newDataRight = false;
		return true;
	}

	return false;
}


void vpStereoCamera::project(const vector<Point3f>& worldPoints, bool isLeft, vector<Point2f>& coords)
{
	Mat pts = Mat(worldPoints);
	coords.resize(worldPoints.size());

	if (!isLeft)
	{
		// transform to other coord system
		vector<Point3f> wpRight;
		for (int i=0; i<worldPoints.size(); i++)
			wpRight.push_back(LtoR(worldPoints[i]));

		pts = Mat(wpRight);
		cvProjectPoints2(&(CvMat)pts, &(CvMat)rotV_L, &(CvMat)trans_L, &(CvMat)intrinsic_R, &(CvMat)distortion_R, &(CvMat)Mat(coords));
	}
	else
	{
		cvProjectPoints2(&(CvMat)pts, &(CvMat)rotV_L, &(CvMat)trans_L, &(CvMat)intrinsic_L, &(CvMat)distortion_L, &(CvMat)Mat(coords));
	}
}

void vpStereoCamera::unproject(const vector<Point2f>& coords, bool isLeft, vector<Point3f>& worldPoints)
{
	// make a mat for input points
	Mat cMat = Mat(coords);

	Mat intr = isLeft?intrinsic_L:intrinsic_R;
	Mat dist = isLeft?distortion_L:distortion_R;

	// undistort points, so that a proper perspective transform will recover our original points
	vector<Point2f> ptsUndist(coords);
	ptsUndist.resize(coords.size());

	cv::undistortPoints(cMat, Mat(ptsUndist), intr, dist);

	// loop through and unproject our points
	for (int i=0; i<ptsUndist.size(); i++)
	{
		Point3f pt;
		pt.x = ptsUndist[i].x;
		pt.y = ptsUndist[i].y;
		pt.z = 1;

		// transform if necessary
		if (!isLeft)
			pt = RtoL(pt);

		worldPoints.push_back(pt);
	}
}

Point3f vpStereoCamera::LtoR(Point3f pt)
{
	return coordinateTransform(pt, rot_R, trans_R);
}

Point3f vpStereoCamera::RtoL(Point3f pt)
{
	return unCoordinateTransform(pt, rot_R, trans_R);
}


void vpStereoCamera::loadCalibration()
{
	cout << "Loading calibration data..." << endl;

	// load all of our matrices, if we can
	try {
		FileStorage fs(VP_CALIB_FILE, FileStorage::READ);

		fs["intr_L"] >> intrinsic_L;
		fs["intr_R"] >> intrinsic_R;
		fs["dist_L"] >> distortion_L;
		fs["dist_R"] >> distortion_R;
		
		Mat rotRtmp;
		fs["rot"] >> rotRtmp;
		fs["trans"] >> trans_R;

		rotV_L = Mat (3, 1, CV_32FC1);
		rotV_L.setTo(0);
		trans_L = Mat(3, 1, CV_32FC1);
		trans_L.setTo(0);
		cv::setIdentity(rot_L);

		//CvMat* rotTmp = (CvMat*)cvLoad( "cam/rot.xml" );
		CvMat* rotTmp = &(CvMat)rotRtmp;
		CvMat* rotRold = cvCreateMat(3, 1, CV_32FC1);

		// need to convert loaded rotation matrix into rodrigues form or whatevs
		// this function seems to crash, use the old one instead
		//cv::Rodrigues(rotMat, rot_R);
		cvRodrigues2(rotTmp, rotRold);

		rotV_R		= Mat(rotRold, true);
		rot_R		= Mat(rotTmp, true);

		rCamPos = -Point3f(trans_R.at<float>(0,0), trans_R.at<float>(1,0), trans_R.at<float>(2,0));

		cout << "Calibration data loaded." << endl;
	}
	catch (...) {
		cout << "Calibration data failed to load." << endl;
	}
}


vpStereoCamera::~vpStereoCamera()
{
	//Shut down devices properly
	VI.stopDevice(deviceLeft);
	VI.stopDevice(deviceRight);
}
