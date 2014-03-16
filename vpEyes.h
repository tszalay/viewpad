#include "stdafx.h"

using namespace std;
using namespace cv;


#ifndef __VPEYES_H__
#define __VPEYES_H__

// a class for holding on to 2D eye subimages, data, and associated methods
struct vpEye2
{
	// these two are related, but offset has to be exact integer
	Point offset;		// offset of eye subimage with respect to the main image
	Point2f _offsetf;	// floating-point version of offset, but still an integer
	Point2f center;		// floating-point detected eye center

	Mat img;			// 1-channel, actual size image
	Mat imgDebug;		// 3-channel, 4x size debug image, points to external data
	Mat imgSwap;		// 3-channel, temporary image for color conversion -> scaling, internal

	// found glint positions, in main image space
	vector<Point2f> glints;

	// best-guess pupil center and approx. radius
	Point2f pupilCenter;
	float pupilRadius;
	// pupil ellipse fit
	RotatedRect pupilEllipse;



	// find the glints, using star detector-fu
	// returns # of glints found
	int findGlints(CvStarDetectorParams params);

	// draw glints on debug images
	void drawGlints();


	// find the rough centers of pupils using star detector
	void findPupilRough();

	// hone in on centers using RANSAC ellipse fit algorithm
	void refinePupil();

	// draw the current best-guess pupil
	void drawPupil();


	// set eye subimages, from a given camera image
	void setImages(const Mat& imgMain)
	{	
		Mat src = imgMain(Rect(offset.x,offset.y,VP_SUBSIZE,VP_SUBSIZE));
		src.copyTo(img);
		cv::cvtColor(src,imgSwap,CV_GRAY2RGB);
		cv::resize(imgSwap,imgDebug,imgDebug.size());
	}

	// set offset, given a center
	void setCenter(Point2f& newcenter)
	{ 
		center = newcenter;
		offset = Point((int)center.x, (int)center.y) - Point(VP_SUBSIZE/2,VP_SUBSIZE/2);
		// note that this should still be roughly integer valued
		_offsetf = Point2f(offset);
	}
	void setCenter(Point& newcenter)
	{
		setCenter(Point2f(newcenter));
	}

	// silly old constructors
	vpEye2()
	{
		img = Mat(VP_SUBSIZE, VP_SUBSIZE, CV_8UC1);
		imgDebug = Mat(VP_SUBSIZE*VP_EYE_DBG_SCL, VP_SUBSIZE*VP_EYE_DBG_SCL, CV_8UC3);
	}

	vpEye2(Mat& imgdbg)
	{
		img = Mat(VP_SUBSIZE, VP_SUBSIZE, CV_8UC1);
		imgSwap = Mat(VP_SUBSIZE, VP_SUBSIZE, CV_8UC3);
		imgDebug = imgdbg;
	}
};

// our container struct for all the relevant pointers to data
struct CorneaData
{
	vpStereoCamera* cam;
	Point3f camPos[2];
	Point3f ledPos[N_LED];

	// number of glints in the left and right images
	int nglints[2];
};

// the class for holding on to 3d eye data
struct vpEye3
{
	


	// pointers to the two Eye2s belonging to this 3d eye, one from each camera
	vpEye2* eyes2[2];

	// 3d position of cornea center, calculated using levmar
	Point3f corneaCenter;

	// 2d positions of cornea centers
	Point2f corneaPos2[2];

	// vector of glints used for corneas in final detection
	vector<Point2f> corneaGlints[2];

	// the 3d position of the pupil in this eye, refracted n stuff
	Point3f pupilCenter;

	// the 2d screen position of this eye's gaze
	Point2f screenPos;


	// temporary var - last cornea data struct
	CorneaData lastData;

	// compute cornea center from glint locations
	// return min-squared error
	// don't need to pass args - we have eye pointers
	float findCorneaCenter();

	// draw on eye's debug images
	void drawCorneaCenter();


	// compute the 3d position of the pupil in this eye, refracted n stuff
	// from the two 2d pupil centers
	void findPupilCenter();

	// and finally get the screen gaze point of this eye
	void findScreenPoint();

		
	vpEye3()
	{}

	vpEye3(vpEye2* eye1, vpEye2* eye2)
	{
		eyes2[0] = eye1;
		eyes2[1] = eye2;
	}
};

#endif