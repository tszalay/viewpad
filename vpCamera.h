#include "stdafx.h"

#ifndef __VPCAMERA_H__
#define __VPCAMERA_H__

//#define DEF_WIDTH 1600
//#define DEF_HEIGHT 1200
#define DEF_WIDTH 800
#define DEF_HEIGHT 600
#define DEF_FPS 30

class vpCamera
{
public:
	bool init();  // returns whether successfully found camera
	bool init(int w, int h); // override for resolution

	// check for new data, return true if any available
	bool pollCamera();

	// get pointer to image (that is not meant to be changed)
	const IplImage* getImage()
	{ return img; }

	// get pointer to image (that is not meant to be changed)
	const IplImage* getGray()
	{ return imgGray; }

	// project and unproject points to/from screen
	vector2 project(vector3 worldPoint);
	// note that this one is a ray from the camera position
	vector3 unproject(vector2 screenPoint);
	
	// set verbosity level
	void setVerbose(bool _verbose)
	{ 
		this->verbose = _verbose;
		VI.setVerbose(_verbose);
	}

	// show the camera settings window
	void showSettings()
	{ VI.showSettingsWindow(device); }

	vpCamera()
	{
		verbose = true;
		VI.setVerbose(true);
	}

	~vpCamera();

private:
	// set up transform matrices
	void initMatrices();

	// internal videoInput object
	videoInput VI;

	// device number
	int device;

	// camera dimensions
	int width;
	int height;
	// and number of pixels
	int imgSize;

	// internal transform matrices
	matrix44 mViewport;
	matrix44 mProjection;
	matrix44 mView;

	// internal buffer
	unsigned char* buffer;

	// internal IplImage
	IplImage* img;
	// grayscale IplImage
	IplImage* imgGray;

	// display debug info in console/log
	bool verbose;
};

#endif