#include "stdafx.h"
#include "vpProcess.h"
#include "vpPointDetector.h"

using namespace std;
using namespace cv;

CvHaarClassifierCascade* vpProcess::haarLeft = NULL;
CvHaarClassifierCascade* vpProcess::haarRight = NULL;
CvHaarClassifierCascade* vpProcess::haarBoth = NULL;
CvMemStorage* vpProcess::storage = NULL;


bool vpProcess::findEyes(int index)
{
	Mat& img = this->subImgOut[index];
	CvSeq* eyesL = cvHaarDetectObjects(&(CvMat)img, haarLeft, storage, 1.2, 4, 0, cvSize(2*VP_EYEMIN,VP_EYEMIN));
	CvSeq* eyesR = cvHaarDetectObjects(&(CvMat)img, haarRight, storage, 1.2, 4, 0, cvSize(2*VP_EYEMIN,VP_EYEMIN));

	Rect r;
    // first, find the rectangle of one classifier with largest min/max range
	double maxRange = 0;
	Rect maxLeft = Rect(0,0,0,0);
	Point2i ptLeft = Point2i(-1, -1);

	for(int i = 0; i < (eyesL ? eyesL->total : 0); i++ ) 
	{
		r = Rect(*(CvRect*)cvGetSeqElem( eyesL, i ));
		Mat subimg = img(r);

		double mn, mx;
		Point pt;
		cv::minMaxLoc(subimg, &mn, &mx, 0, &pt);

		if (fabs(mx-mn) > maxRange)
		{
			maxRange = fabs(mx-mn);
			maxLeft = r;
			ptLeft = pt;
		}
	}

	maxRange = 0;
	Rect maxRight = Rect(0,0,0,0);
	Point2i ptRight = Point2i(-1, -1);

	// now do the same for the other eye, with the added provision of non-intersection
	for(int i = 0; i < (eyesR ? eyesR->total : 0); i++ ) 
	{
		r = Rect(*(CvRect*)cvGetSeqElem( eyesR, i ));
		Rect rInt = r & maxLeft;

		// skip if overlapping
		if (rInt.area() != 0)
			continue;
		
		double mn, mx;
		Mat subimg = img(r);

		Point pt;
		cv::minMaxLoc(subimg, &mn, &mx, 0, &pt);

		if (fabs(mx-mn) > maxRange)
		{
			maxRange = fabs(mx-mn);
			maxRight = r;
			ptRight = pt;
		}
	}

	// now make sure, since we are operating on the shrunken images, to scale everything back up to full-size
	Point eyeL = VP_MAIN_DBG_SCL*(maxLeft.tl()+ptLeft);
	Point eyeR = VP_MAIN_DBG_SCL*(maxRight.tl()+ptRight);

	if (maxLeft.x > maxRight.x)
	{
		this->eyes2[index][0].setCenter(eyeL);
		this->eyes2[index][1].setCenter(eyeR);
	}
	else
	{
		this->eyes2[index][0].setCenter(eyeR);
		this->eyes2[index][1].setCenter(eyeL);
	}

	cvClearMemStorage(storage);

	// check whether both were correctly detected
	this->eyesFound[index] = (ptLeft.x >=0 && ptRight.x >= 0);

	return this->eyesFound[index];
}


// sanity check for relative eye positions, discard if head is interpreted as way too tilted
bool vpProcess::checkEyes()
{
	for (int i=0; i<2; i++)
	{
		Point2f pt = this->eyes2[i][1].center - this->eyes2[i][0].center;
		// check tilt using tangent
		this->eyesFound[i] &= (fabs(pt.y/pt.x) < VP_EYEMAXROT);
		// not too close to edge also. causes problems later on
		for (int j=0; j<2; j++)
			this->eyesFound[i] &= Rect(VP_SUBSIZE, VP_SUBSIZE, camImageSize.width-2*VP_SUBSIZE, camImageSize.height-2*VP_SUBSIZE).contains(this->eyes2[i][j].center);
	}

	return (this->eyesFound[0] && this->eyesFound[1]);
}