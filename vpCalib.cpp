#include "stdafx.h"
#include "vpCalib.h"
#include <sstream>

using namespace std;
using namespace cv;

void vpCalib::addImage()
{
	images[0].push_back(cam->getImageLeft().clone());
	images[1].push_back(cam->getImageRight().clone());

	cv::resize(images[0].back(), subImg[1][0], subImg[1][0].size());
	cv::resize(images[1].back(), subImg[1][1], subImg[1][1].size());

	for (int i=0; i<2; i++)
	{
		vector<Point2f> corners;
		CvSize board_sz = cvSize( VP_CALIB_BWIDTH, VP_CALIB_BHEIGHT );
		int found = cv::findChessboardCorners( subImg[1][i], board_sz, corners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
		cv::drawChessboardCorners( subImg[1][i], board_sz, corners, (corners.size() == board_sz.height*board_sz.width));
	}
}

void vpCalib::updateImage()
{
	if (capturing)
	{
		cv::resize(cam->getImageLeft(), subImg[0][0], subImg[1][0].size());
		cv::resize(cam->getImageRight(), subImg[0][1], subImg[1][1].size());
	}
	else
	{
		// copy currently selected images
		for (int i=0; i<2; i++)
		{
			Mat& img = this->images[i][curIndex];
			Rect imgRect = Rect(Point(), img.size());
			Size dstSize = subImg[0][i].size();
			// copy into top window
			cv::resize(img, subImg[0][i], dstSize);

			Point offset = Point(dstSize.width/2, dstSize.height/2);
			//and copy the zoomed image as well
			Rect src = Rect(this->zoomCenter[i]-offset, dstSize);
			// check if area is valid
			if ((imgRect & src) != src)
				continue;

			// otherwise copy
			cv::resize(this->images[i][curIndex](src), subImg[1][i], dstSize);

			// if we have any target points up, draw them
			for (int j=0; j<N_TGT; j++)
			{
				// not picked, skip
				if (targetPoints[i][curIndex][j] == Point())
					continue;

				std::string s;
				std::stringstream out;
				out << j+1;
				s = out.str();

				cv::circle(subImg[1][i], targetPoints[i][curIndex][j] + offset - zoomCenter[i], 3, CV_RGB(255,0,0));
				cv::putText(subImg[1][i], out.str(), targetPoints[i][curIndex][j] + offset - zoomCenter[i]+Point(5,-5), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255,0,0));
			}
		}
	}

	cvShowImage(VP_CALIB_TITLE, &(CvMat)fullImg);
}


void vpCalib::next()
{
	if (capturing)
		return;
	if (this->curIndex < this->images[0].size()-1)
		this->curIndex++;
}

void vpCalib::previous()
{
	if (capturing)
		return;
	if (this->curIndex > 0)
		this->curIndex--;
}

void vpCalib::clearpt()
{
	if (capturing)
		return;
	// reset current point in both eyes so it's skippable
	this->targetPoints[0][this->curIndex][this->curTarget] = Point();
	this->targetPoints[1][this->curIndex][this->curTarget] = Point();
}

void vpCalib::onMouse(int evt, int x, int y, int flags, void* param)
{
	vpCalib* calib = (vpCalib*)param;

	switch(evt)
	{
	case CV_EVENT_LBUTTONDOWN:
		if (y < calib->subImg[0][0].rows)
		{
			// update centers of zoomed lower images
		
			if (x < calib->subImg[0][0].cols)
				calib->zoomCenter[0] = Point(x,y)*VP_CAL_DBG_SCL;
			else
				calib->zoomCenter[1] = Point(x-calib->subImg[0][0].cols,y)*VP_CAL_DBG_SCL;
		}
		else
		{
			// or else pick our currently selected points
			// recenter, w.r.t zoomCenter
			if (x < calib->subImg[0][0].cols)
				calib->targetPoints[0][calib->curIndex][calib->curTarget] = Point(x-calib->subImg[0][0].cols/2, y-(3*calib->subImg[0][0].rows)/2)+calib->zoomCenter[0];
			else
				calib->targetPoints[1][calib->curIndex][calib->curTarget] = Point(x-(3*calib->subImg[0][0].cols)/2,y-(3*calib->subImg[0][0].rows)/2)+calib->zoomCenter[1];
		}
		break;
	}
}


void vpCalib::setTarget(int target)
{
	if (target < N_TGT)
		this->curTarget = target;
}


vpCalib::vpCalib(bool full) : capturing(true), curIndex(0), curTarget(0)
{
	this->cam = vpConfig::getCamera();
	this->fullCalib = full;

	int dw=cam->getImageLeft().cols/VP_CAL_DBG_SCL;
	int dh=cam->getImageLeft().rows/VP_CAL_DBG_SCL;
	
	// create full-color images and stuff
	fullImg = Mat(2*dh, 2*dw, CV_8UC3);
	// set the sub-images
	for (int i=0; i<2; i++)
		for (int j=0; j<2; j++)
			subImg[i][j] = fullImg(Rect(j*dw,i*dh,dw,dh));
	// note: we are using full color for drawChessboardCorners and such
	for (int i=0; i<2; i++)
		targetPoints[i] = NULL;
}

vpCalib::~vpCalib()
{
	cv::destroyWindow(VP_CALIB_TITLE);
}