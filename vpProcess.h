#include "stdafx.h"

using namespace std;
using namespace cv;


#ifndef __VPPROCESS_H__
#define __VPPROCESS_H__

#include "vpEyes.h"

class vpProcess
{
private:
	// left and right classifier files
	static CvHaarClassifierCascade* haarLeft;
	static CvHaarClassifierCascade* haarRight;
	static CvHaarClassifierCascade* haarBoth;

	static CvMemStorage* storage;

	// overall image size, for calculations
	Size camImageSize;

public:
	// double subimage, for storing scaled-down versions of whole image to display
	Mat curImgOut;
	// pointers to above image, for each individual camera
	Mat subImgOut[2];

	// large color image holding all four eyes, for debug purposes
	Mat eyesOut;
	
	bool drawDebug;
	bool eyesFound[2];

	// structs containing the 2d eye data
	vpEye2 eyes2[2][2];

	// and the 3d eye data, one for left and one for right
	vpEye3 eyes3[2];


	// load all haar classifier files, calibration params
	static bool init();

	// find the left and right eyes in one of our subImgs using Haar classifier
	bool findEyes(int index);

	// check eye locations for overall sanity testing
	// returns true iff eyes found in all images
	bool checkEyes();
	
	// draw location of things on the main output image
	void drawEyeDebug();





	// draw list of points on an image, for debug
	static void drawPoints(Mat& img, vector<Point2f>& pts, int radius);


	// initialize our class and our output image at 1/16 the size
	vpProcess(const Mat& img) : camImageSize(img.cols,img.rows)
	{
		eyesFound[0] = eyesFound[1] = false;
		drawDebug = true;

		int dw=img.cols/VP_MAIN_DBG_SCL;
		int dh=img.rows/VP_MAIN_DBG_SCL;
		curImgOut = Mat(dh, 2*dw, CV_8UC1);
		for (int i=0; i<2; i++)
			subImgOut[i] = curImgOut(Rect(i*dw,0,dw,dh));

		dw = VP_EYE_DBG_SCL*VP_SUBSIZE;
		eyesOut = Mat(Size(2*dw, 2*dw), CV_8UC3);
		
		// call constructors on eye images
		for (int i=0; i<2; i++)
			for (int j=0; j<2; j++)
				eyes2[i][j] = vpEye2(eyesOut(Rect(j*dw,i*dw,dw,dw)));

		for (int i=0; i<2; i++)
			eyes3[i] = vpEye3(&eyes2[0][i], &eyes2[1][i]);
	}

	// set the output image
	void setOutImage(Mat& img, int index)
	{
		cv::resize(img, subImgOut[index], subImgOut[index].size());
	}

private:
	// create a difference of gaussians filter for gaussian point blob finding
	static Mat createDoG(int w, float s1, float s2);
};

#endif