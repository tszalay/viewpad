#include "stdafx.h"

using namespace cv;

#ifndef _VPCALIB_H_
#define _VPCALIB_H_


// struct containing float x 7
struct MirrorData
{
	Point3f orig;		// position of the origin
	Point3f dir;		// first two: theta, phi of normal
						// z: angle around that axis of v1

	Point3f getNormal()
	{
		// the normal comes from spherical coordinate angles 
		return Point3f(sin(dir.x)*cos(dir.y), sin(dir.x)*sin(dir.y), cos(dir.x));
	}

	// compute calibration pattern axes
	void getVectors(Point3f& vX, Point3f& vY)
	{
		// first, we need to generate two axes of calib pattern in world coords
		Point3f normal = getNormal();
		// compute v0 as the normal x y axis (which will not be colinear)
		Point3f v0 = -VP_CALIB_BDIM*normalize(normal.cross(Point3f(0, 1, 0)));
		// now v1 gives us a third orthogonal axis, now we need to rotate them in that plane
		Point3f v1 = VP_CALIB_BDIM*normalize(normal.cross(v0));
	
		// our x- and y- axes of chessboard, to be added to mdata->orig
		vX = cos(dir.z)*v0 - sin(dir.z)*v1;
		vY = sin(dir.z)*v0 + cos(dir.z)*v1;
	}

	// reflect a ray off of this mirror
	void reflectRay(Point3f& pt0, Point3f& pt1)
	{
		// get our local coordinate axes
		Point3f vX, vY, vZ;
		vZ = getNormal();
		getVectors(vX, vY);
		vX = normalize(vX);
		vY = normalize(vY);
		// find the intersection point
		float u = vZ.dot(orig-pt0)/vZ.dot(pt1-pt0);
		Point3f ptint = pt0 + u*(pt1-pt0);
		// get ray origin representation in plane axes
		Point3f ptnew;
		ptnew.x = vX.dot(pt0 - ptint);
		ptnew.y = vY.dot(pt0 - ptint);
		ptnew.z = vZ.dot(pt0 - ptint);
		Point3f pt0new;
		// and reflect the x and y
		pt0new = -vX*ptnew.x - vY*ptnew.y + vZ*ptnew.z;
		pt1 = pt0new + ptint;
		pt0 = ptint;
	}

	MirrorData() : orig(0, 0, 0.8f), dir(0, 0, 0)
	{}
};


// A class created solely for the purpose of calibration
// Created when calib starts, deleted when it ends
class vpCalib
{
private:
	// arrays of calibration images in left and right 
	vector<Mat> images[2];

	// we will need to access some of these from a static function
	// but a vpCalib static function, so everything can proceed as normal
	vpStereoCamera* cam;

	// main mat, holding 2x2 grid of camera images
	Mat fullImg;

	// indexed images into the above, for convenience
	Mat subImg[2][2];

	// are we capturing?
	bool capturing;

	// do we want to save full calib data, or only target calib?
	bool fullCalib;

	// the current image index we have selected
	int curIndex;

	// the current object location we are setting
	int curTarget;

	// the centers of the two zoomed-in subimages
	Point zoomCenter[2];

	// a dynamically allocated 2D array of Points of minor index 6
	// to hold screen/LED points for each target image
	Point ** targetPoints[2];

	// the vectors of rotation and translation vectors for the images
	vector<Mat> mirrorRot[2];
	vector<Mat> mirrorPos[2];

	// the data from a post-calibration fit to original calibration patterns
	vector<MirrorData> mirrorData;

public:

	// are we in the capturing or the post-capture selection phase?
	bool isCapturing() {return capturing;}

	// update the camera images to output window, show window
	void updateImage();

	// process and add an image pair taken simultaneously with both cameras
	void addImage();

	// compute the calibration matrices from the processed images, with given chessboard data
	// use them to compute the mirror angles in each image as well, for screen corners and LEDs
	bool doCalibrate(int nwidth, int nheight, float blockdim);

	// finally, compute and save LED/screen positions from user-entered data
	void finishCalibrate();

	// interface to advance current image forwards and backwards
	void next();
	void previous();
	// clear currently set points, in case of oopsies
	void clearpt();

	// set which object's position we are selecting
	void setTarget(int target);

	// callback for detecting mouse events
	static void onMouse(int evt, int x, int y, int flags, void* param);

	vpCalib(bool full=true);
	~vpCalib();
};


#endif