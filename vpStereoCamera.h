#include "stdafx.h"

#ifndef __VPSTEREOCAMERA_H__
#define __VPSTEREOCAMERA_H__

using namespace cv;

#define DEF_WIDTH 1600
#define DEF_HEIGHT 1200
//#define DEF_WIDTH 800
//#define DEF_HEIGHT 600
#define DEF_FPS 30

class vpStereoCamera
{
public:
	bool init();  // returns whether successfully found cameras
	bool init(int w, int h); // override for resolution set

	// public so we can reload when done calibrating
	void loadCalibration(); // load our calibration data
	
	// check for new data, return true if new image for both cameras
	bool pollCamera();

	// get pointer to images (that is not meant to be changed)
	const cv::Mat getImageLeft()
	{ return imgLeft; }
	const cv::Mat getImageRight()
	{ return imgRight; }

	// get pointer to converted grayscale images (that is not meant to be changed)
	const cv::Mat getGrayLeft()
	{ return imgGrayLeft; }
	const cv::Mat getGrayRight()
	{ return imgGrayRight; }

	// project and unproject points to/from screen
	void project(const vector<Point3f>& worldPoints, bool isLeft, vector<Point2f>& coords);

	// note that this one is a ray from a particular camera position
	void unproject(const vector<Point2f>& coords, bool isLeft, vector<Point3f>& worldPoints);

	// transform from left-to-right coordinate system and vice versa
	Point3f LtoR(Point3f pt);
	Point3f RtoL(Point3f pt);

	// get right camera pos
	Point3f RPos()
	{ return rCamPos; }
	
	// set verbosity level
	void setVerbose(bool _verbose)
	{ 
		this->verbose = _verbose;
		VI.setVerbose(_verbose);
	}

	// show the camera settings window
	void showSettingsLeft()
	{ VI.showSettingsWindow(deviceLeft); }

	void showSettingsRight()
	{ VI.showSettingsWindow(deviceRight); }

	vpStereoCamera() : newDataLeft(false), newDataRight(false)
	{
		verbose = true;
		VI.setVerbose(true);
	}

	~vpStereoCamera();

private:

	// internal videoInput object
	videoInput VI;

	// device numbers
	int deviceLeft;
	int deviceRight;

	// camera dimensions
	int width;
	int height;
	// and number of pixels
	int imgSize;

	// camera projection and distortion data, loaded from file
	cv::Mat intrinsic_L;
	cv::Mat intrinsic_R;

	cv::Mat distortion_L;
	cv::Mat distortion_R;

	// rotation and translation info. identity for left camera.
	cv::Mat rot_L;
	cv::Mat rot_R;
	cv::Mat rotV_L;
	cv::Mat rotV_R;
	cv::Mat trans_L;
	cv::Mat trans_R;

	// position of right camera, in left's frame
	Point3f rCamPos;

	// internal buffer
	unsigned char* buffer;

	// internal IplImages
	cv::Mat imgLeft;
	cv::Mat imgRight;
	// grayscale IplImages
	cv::Mat imgGrayLeft;
	cv::Mat imgGrayRight;

	// which cameras have we received new data for
	bool newDataLeft;
	bool newDataRight;

	// display debug info in console/log
	bool verbose;
};

#endif