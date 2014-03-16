#include "stdafx.h"
#include "vpCamera.h"

using namespace cml;

bool vpCamera::init()
{
	//Prints out a list of available devices and returns num of devices found
	int numDevices = VI.listDevices();	
	
	device = 0;  //this could be any deviceID that shows up in listDevices
	
	//setup the first device - there are a number of options:
	VI.setIdealFramerate(device, DEF_FPS);
	while(!VI.setupDevice(device, DEF_WIDTH, DEF_HEIGHT)) //setup the first device with the default settings;
	{
		// try the next device
		device++;
		// if there are no more left to try...
		if (device == numDevices)
			return false;
	}

	// set our various parameters
	width 	= VI.getWidth(device);
	height 	= VI.getHeight(device);
	imgSize	= VI.getSize(device);
	if (verbose)
		printf("Device initialized at %dx%d, img=%d bytes\n",width,height,imgSize);

	// set some filter parameters as we want them to be
	VI.setVideoSettingFilterPct(device, VI.propSharpness, 1.0);
	//VI.setVideoSettingCameraPct(device, VI.propZoom, 100);
	
	// create the internal image
	buffer = new unsigned char[imgSize];
	img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	imgGray = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	// don't forget to setup the matrices
	this->initMatrices();

	return true;
}

bool vpCamera::pollCamera()
{
	// check for a new frame, retrieve, and set OpenCV image
	if (VI.isFrameNew(device)){
		VI.getPixels(device, buffer, false, true);
		cvSetData(img, buffer, img->widthStep);
		cvCvtColor(img, imgGray, CV_BGR2GRAY);
		return true;
	}
	return false;
}


vector2 vpCamera::project(vector3 worldPoint)
{
	vector3 pt = project_point(this->mView,this->mProjection,this->mViewport,worldPoint);
	return vector2(pt[0], pt[1]);
}

vector3 vpCamera::unproject(vector2 screenPoint)
{
	vector3 dir = unproject_point(this->mView,this->mProjection,this->mViewport,vector3(screenPoint[0],screenPoint[1],0));
	return dir.normalize();
}


void vpCamera::initMatrices()
{
	// projection matrix with ~45 deg FOV in x, near->far 0.01m to 2m clip
	matrix_perspective_xfov(this->mProjection, 0.725f, (float)this->width/(float)this->height, 0.01f, 2.0f, right_handed, z_clip_neg_one);
	// viewport matrix
	matrix_viewport<float,  cml::fixed<4,4>, col_basis, col_major>(this->mViewport, 0.f, this->width, 0.f, this->height, cml::z_clip_neg_one);
	// and lastly camera world position (aka view matrix)
	matrix_look_at_RH(this->mView, vpConfig::camPos, vpConfig::camPos+vpConfig::camDir,vector3(0,1,0));
}


vpCamera::~vpCamera()
{
	//Shut down devices properly
	VI.stopDevice(device);
}




/*
void showCaptureProperties(CvCapture* cap){
	int  w,h,f;
	w=(int)cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_WIDTH);
	h=(int)cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_HEIGHT);
	f=(int)cvGetCaptureProperty(cap,CV_CAP_PROP_FPS);
	printf("Capture properties (widthxheight - frames): %dx%d - %d\n",w,h,f);
}

void printCaptureSettings(videoInput &VI, int device)
{
	int settings[17];
	char* setName[] = {"Backlight", "Brightness", "ColorEnable", "Contrast", "Exposure", "Focus", "Gain", "Gamma", "Hue", "Iris",
						"Pan", "Roll", "Saturation", "Sharpness", "Tilt", "WhiteBalance", "Zoom"};

	settings[0] = VI.propBacklightCompensation;
	settings[1] = VI.propBrightness;
	settings[2] = VI.propColorEnable;
	settings[3] = VI.propContrast;
	settings[4] = VI.propExposure;
	settings[5] = VI.propFocus;
	settings[6] = VI.propGain;
	settings[7] = VI.propGamma;
	settings[8] = VI.propHue;
	settings[9] = VI.propIris;
	settings[10] = VI.propPan;
	settings[11] = VI.propRoll;
	settings[12] = VI.propSaturation;
	settings[13] = VI.propSharpness;
	settings[14] = VI.propTilt;
	settings[15] = VI.propWhiteBalance;
	settings[16] = VI.propZoom;

	VI.setVerbose(false);

	for (int i=0; i<17; i++)
	{
		long min=-1, max=-1, SteppingDelta=-1, currentValue=-1, flags=-1, defaultValue=-1;

		
		VI.getVideoSettingCamera(device, settings[i], min, max, SteppingDelta, currentValue, flags, defaultValue);
		if (min != max)
		{
			printf("Camera %d/%d (%s): MIN=%ld | MAX=%ld | DELTA=%ld | CUR=%ld | FLAGS=%ld | DEF=%d\n",
				i, settings[i], setName[i], min, max, SteppingDelta, currentValue, flags, defaultValue);
		}
		else
		{
			if (VI.getVideoSettingFilter(device, settings[i], min, max, SteppingDelta, currentValue, flags, defaultValue))
				printf("Filter %d/%d (%s): MIN=%ld | MAX=%ld | DELTA=%ld | CUR=%ld | FLAGS=%ld | DEF=%d\n",
					i, settings[i], setName[i], min, max, SteppingDelta, currentValue, flags, defaultValue);
			else
				printf("Setting %d/%d : %s not supported.\n", i, settings[i], setName[i]);
		}
	}

	VI.setVerbose(true);
}
*/