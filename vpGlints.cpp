#include "stdafx.h"
#include "vpProcess.h"
#include "vpPointDetector.h"

using namespace std;
using namespace cv;

// First pass, finds all of the gaussian point blobs in image
int vpEye2::findGlints(CvStarDetectorParams params)
{
	double min, max;

	vpPointDetector det = vpPointDetector(this->img.cols,this->img.rows);

	det.maxSize = params.maxSize;
	det.responseThreshold = params.responseThreshold;
	det.suppressNonmaxSize = params.suppressNonmaxSize;
	det.checkMin = false;

	vector<CvStarKeypoint> keypoints = det.getKeypoints(&(IplImage)this->img);

	this->glints.clear();

	// give us all the glints we found, but not in useless eye coordinates
	for (int i=0; i<keypoints.size(); i++)
		glints.push_back(Point2f(keypoints[i].pt.x+this->offset.x, keypoints[i].pt.y+this->offset.y));

	return glints.size();
}


// and draw on da debug image if we feel like it
void vpEye2::drawGlints()
{
	for (int i=0; i<this->glints.size(); i++)
	{
		Point2f pt = (glints[i] - this->_offsetf)*VP_EYE_DBG_SCL;
		cv::circle(this->imgDebug, Point(pt.x,pt.y), 15, CV_RGB(255,0,0));
	}
}