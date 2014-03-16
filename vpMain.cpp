#include "stdafx.h"
#include "vpProcess.h"
#include "vpLed.h"
#include "vpCalib.h"

#include <algorithm>

using namespace std;
using namespace cv;

int main(int argc, char ** argv)
{
	vpStereoCamera cam;
	vpLed leds;

	// startup the stereo cameras
	if (!cam.init())
	{
		fprintf(stderr, "Camera initialization failed. Exiting.\n");
		return 1;
	}
	vpConfig::setCamera(&cam);
	vpConfig::init();

	// connect with the LEDs and turn them on via serial
	leds.open(L"COM3");
	leds.on();

	// load Haar classifiers
	if (!vpProcess::init())
	{
		fprintf(stderr, "Failed to load Haar classifiers. Exiting.\n");
		return 1;
	}

	// initialize tracker output images
	vpProcess* process = new vpProcess(cam.getImageLeft());
	// calibration handling struct
	vpCalib* calib = NULL;
	// did we just signal to capture an image?
	bool capImage = false;

	while (1)
	{
		int c = cvWaitKey(10);
		if (c == 27) break;

		if (c == 'd')
		{
			if (!calib)
			{
				// start
				calib = new vpCalib(false);
			}
		}

		if (c == 'c')
		{
			if (!calib)
			{
				// start
				calib = new vpCalib();
			}
			else
			{
				if (calib->isCapturing())
				{
					// calculate positions of mirror in captured images
					if (!calib->doCalibrate(VP_CALIB_BWIDTH,VP_CALIB_BHEIGHT,VP_CALIB_BDIM))
					{
						// calibration failed for some reason
						delete calib;
						calib = NULL;
					}
				}
				else
				{
					// do final LED/screen position calculations
					calib->finishCalibrate();
					delete calib;
					calib = NULL;
				}
			}
		}
		if (c == 'n' && calib)
			calib->next();
		if (c == 'p' && calib)
			calib->previous();
		if (c == 'x' && calib)
			calib->clearpt();
		if (c == ' ')
			capImage = true;
		if (c == '[')
			leds.dim();
		if (c == ']')
			leds.on();
		if (c == 'j')
			cam.showSettingsLeft();
		if (c == 'k')
			cam.showSettingsRight();

		// ascii codes magicks
		if (c >= '1' && c <= '9' && calib)
			calib->setTarget(c - '1');
		
		if (cam.pollCamera())
		{
			Mat img[2];
			img[0] = cam.getGrayLeft();
			img[1] = cam.getGrayRight();

			vpConfig::tick("Img Acq");

			// are we calibrating? then calibrate, dammit...
			if (calib)
			{
				// update with live image if capturing, or else with stored images for static
				calib->updateImage();

				// grab an image if spacebar pressed
				if (capImage && calib->isCapturing())
				{
					capImage = false;
					calib->addImage();
				}

				// skip the rest of the tracking stuff
				continue;
			}
			
			// check to make sure we have found eyes for both cameras
			for (int i=0; i<2; i++)
			{
				// set part of the image
				process->setOutImage(img[i], i);
				if (process->eyesFound[i])
					continue;
				// and now find them eyes
				process->findEyes(i);
			}

			// draw where our found eyes are, if found
			process->drawEyeDebug();
			cvShowImage("Camera Images", &(CvMat)process->curImgOut);

			// verify eye locations for consistency
			if (!process->checkEyes())
				continue;
	
			vpConfig::tick("Img Manip");

			// found eyes, now locate glints
			for (int i=0; i<2; i++)
			{
				for (int j=0; j<2; j++)
				{
					process->eyes2[i][j].setImages(img[i]);
					
					CvStarDetectorParams params = cvStarDetectorParams(10, 15, 10, 8, 8);
					int nfound = process->eyes2[i][j].findGlints(params);

					if (process->drawDebug)
						process->eyes2[i][j].drawGlints();

					if (nfound == 0) 
						process->eyesFound[i] = false;
				}
			}

			vpConfig::tick("Glints");


			// still ok to continue?
			if (!process->checkEyes())
				continue;

			// find corneas, if we can
			for (int i=0; i<2; i++)
			{
				process->eyes3[i].findCorneaCenter();
				process->eyes3[i].drawCorneaCenter();
			}

			vpConfig::tick("Cornea");

			// and pupils in each image
			for (int i=0; i<2; i++)
			{
				for (int j=0; j<2; j++)
				{
					process->eyes2[i][j].findPupilRough();
					process->eyes2[i][j].refinePupil();
					process->eyes2[i][j].drawPupil();
				}
			}

			vpConfig::tick("Pupil");

			// now find the screen coords!
			for (int i=0; i<2; i++)
			{
				process->eyes3[i].findPupilCenter();
				process->eyes3[i].findScreenPoint();
			}

			Point2f avgPos = 0.5f*(process->eyes3[0].screenPos+process->eyes3[1].screenPos);
			//SetCursorPos((int)avgPos.x,(int)avgPos.y);

			vpConfig::tick("Screen");

			cvShowImage("Eyes Tracked", &(CvMat)process->eyesOut);

			// now lastly update eye positions
			for (int i=0; i<2; i++)
				for (int j=0; j<2; j++)
					process->eyes2[i][j].setCenter(process->eyes3[j].corneaPos2[i]);
		}
	}

	vpConfig::close();

	return 0;
}