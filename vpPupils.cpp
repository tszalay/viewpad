#include "stdafx.h"
#include <algorithm>
#include "vpProcess.h"
#include "vpPointDetector.h"

bool pupilSort (CvStarKeypoint i,CvStarKeypoint j) { return (i.response<j.response); }

void vpEye2::findPupilRough()
{
	vpPointDetector det = vpPointDetector(img.cols, img.rows);

	det.checkMin = true;
	det.checkMax = false;
	det.responseThreshold = 18;
	det.maxSize = 20;
	det.suppressNonmaxSize = 5;
	det.lineThresholdBinarized = 10;
	det.lineThresholdProjected = 8;

	vector<CvStarKeypoint> pupils = det.getKeypoints(&(IplImage)this->img);
	vector<Point2f> finalPupils;

	const int pupilMin = 8;

	for (vector<CvStarKeypoint>::iterator kp = pupils.begin(); kp<pupils.end();)
	{
		if (kp->size < pupilMin)
			kp=pupils.erase(kp);
		else
			kp++;
	}
	sort(pupils.begin(),pupils.end(),pupilSort);
	
	if (pupils.size() == 0)
		return;

	this->pupilCenter = Point2f(pupils[0].pt.x,pupils[0].pt.y) + this->_offsetf;
	this->pupilRadius = pupils[0].size;
}

void vpEye2::refinePupil()
{
	Point pmin;
	double dmin, dmax;
	// first, median blur, to preserve edges
	cv::medianBlur(this->img,this->img,5);
	// find minimum value, point
	cv::minMaxLoc(this->img, &dmin, &dmax, &pmin);
	// and do a flood fill to find pupil region based on this, finding rect
	Rect r;
	cv::floodFill(this->img, pmin, 0, &r, 10, 15, 4 | FLOODFILL_FIXED_RANGE);
	// set ROI to pupil only
	Mat pupilImg = this->img(r);
	// threshold, so that we can have a proper binary image
	cv::threshold(pupilImg, pupilImg, 0, 255, cv::THRESH_BINARY_INV);
	// now find the contour we just drew
	// want to get hierarchies, so we can know which one is pupil
	vector<vector<Point> > contours;
	cv::findContours(pupilImg, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, r.tl());
	if (contours.size() == 0) return;

	vector<Point> hull;
	cv::convexHull(Mat(contours.front()), hull);

	if (hull.size() < 6)
		return;
	
	this->pupilEllipse = cv::fitEllipse(Mat(hull));
	this->pupilCenter = Point2f(pupilEllipse.center) + this->_offsetf;
	this->pupilRadius = 0.25f*(pupilEllipse.size.width + pupilEllipse.size.height);
}


void vpEye2::drawPupil()
{
	Point2f pt = VP_EYE_DBG_SCL*(this->pupilCenter - this->_offsetf);
	Point p = Point(pt);
	if (this->pupilCenter != Point2f())
		cv::circle(this->imgDebug, p, this->pupilRadius*VP_EYE_DBG_SCL, CV_RGB(255,255,0));
}


void vpEye3::findPupilCenter()
{
	// alright, first, we get the two rays from each 2d pupil
	Point3f pRay[2];
	// centers
	Point3f p0[2];

	p0[0] = Point3f();
	p0[1] = vpConfig::getCamera()->RPos();

	for (int i=0; i<2; i++)
	{
		vector<Point2f> pts;
		vector<Point3f> outpts;
		pts.push_back(this->eyes2[i]->pupilCenter);

		vpConfig::getCamera()->unproject(pts, (i==0), outpts);
		pRay[i] = normalize(outpts[0]);
	}

	// start with empty one
	Point3f pCenter = Point3f();
	int n = 0;

	// now compute intersections with cornea
	for (int i=0; i<2; i++)
	{
		float a = pRay[i].dot(pRay[i]);
		Point3f s = p0[i] - corneaCenter;
		float b = 2*pRay[i].dot(s);
		float c = s.dot(s) - vpConfig::corneaRadius*vpConfig::corneaRadius;

		// did it intersect with cornea sphere?
		if ((b*b - 4*a*c) <= 0)
			continue;

		// get closer (smaller) intersection
		float t = (-b - sqrt(b*b-4*a*c))/(2*a);
		pCenter = pCenter + p0[i] + pRay[i]*t;
		n++;
	}

	if (n == 0)
	{
		cout << "Pupil screen coord error." << endl;
		return;
	}
	cout << "Center found with " << n << " points." << endl;
	pCenter = pCenter * (1.0f/n);

	// reproject onto surface of sphere, post-averaging
	pCenter = corneaCenter + normalize(pCenter-corneaCenter)*vpConfig::corneaRadius;

	pupilCenter = pCenter;
}