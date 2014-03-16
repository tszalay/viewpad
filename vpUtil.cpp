#include "stdafx.h"
#include "vpProcess.h"

using namespace std;
using namespace cv;

// Create a difference of gaussian filter
// Normalized to return 0 for constant color, 1 for perfect match
// params: total kernel width, small sigma, large sigma
Mat vpProcess::createDoG(int width, float s1, float s2)
{
    CvMat* kernel = cvCreateMat(width, width, CV_32FC1);
    cvZero(kernel);
	
	float x0 = 0.5f*(float)(width-1);

	float norm1=0.0f;
	float norm2=0.0f;

	// compute the normalization factors
	for (int x=0; x<width; x++)
		for (int y=0; y<width; y++)
			norm1 += exp(-((x-x0)*(x-x0)+(y-x0)*(y-x0))/(2*s1*s1));

	for (int x=0; x<width; x++)
		for (int y=0; y<width; y++)
			norm2 += exp(-((x-x0)*(x-x0)+(y-x0)*(y-x0))/(2*s2*s2));

	// normalize to 1 total
	norm1 *= 1.0f;
	norm2 *= 1.0f;

	// create the filter
	for (int x=0; x<width; x++)
		for (int y=0; y<width; y++)
			kernel->data.fl[y*width+x] = exp(-((x-x0)*(x-x0)+(y-x0)*(y-x0))/(2*s1*s1))/norm1 - exp(-((x-x0)*(x-x0)+(y-x0)*(y-x0))/(2*s2*s2))/norm2;

	for (int x=0; x<width; x++)
		for (int y=0; y<width; y++)
			kernel->data.fl[y*width+x] = -1.0f/(width*width);
	int w4 = width/4;
	for (int x=w4; x<width-w4; x++)
		for (int y=w4; y<width-w4; y++)
			kernel->data.fl[y*width+x] += 1.0f/((width-2*w4)*(width-2*w4));


    return kernel;
}

// draws the specified list of points on the given image with the given radius
void vpProcess::drawPoints(Mat& img, vector<Point2f>& pts, int radius)
{
	for (int i=0; i<(int)pts.size(); i++)
		cv::circle( img, Point(cvRound(pts[i].x),cvRound(pts[i].y)), radius, CV_RGB(255,255,255), 1, 8, 0 );
}

// draws debug images on the output while searching for (or having found) eyes
void vpProcess::drawEyeDebug()
{
	for (int i=0; i<2; i++)
	{
		if (!this->eyesFound[i])
			continue;
		// draw subeye rect
		for (int j=0; j<2; j++)
		{
			Point pt1 = Point(this->eyes2[i][j].offset.x/VP_MAIN_DBG_SCL,this->eyes2[i][j].offset.y/VP_MAIN_DBG_SCL);
			Point pt2 = pt1 + Point(VP_SUBSIZE/VP_MAIN_DBG_SCL, VP_SUBSIZE/VP_MAIN_DBG_SCL);
			cv::rectangle(this->subImgOut[i], pt1, pt2, CV_RGB(255,255,255));
		}
	}
}