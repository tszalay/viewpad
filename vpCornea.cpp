#include "stdafx.h"
#include <algorithm>
#include "vpProcess.h"

using namespace std;
using namespace cv;


// sort by 2d x-coordinate
bool pt2sort (Point2f i,Point2f j) { return (i.x<j.x); }

// count the number of bits in a number
int bitcount (unsigned int n)  {
   int count = 0 ;
   while (n)  {
      count++ ;
      n &= (n - 1) ;
   }
   return count;
}


// this is the function called by Levenberg-Marquardt, to minimize error
// p is the array of parameters (input xyz), of dimension m
// hx is output screen coord xy of projected points, total length n
void reprojectError(float *p, float *hx, int m, int n, void *adata)
{
	CorneaData *data = (CorneaData*)adata;

	// access p as a point3f
	Point3f* cornea = (Point3f*)p;
	// and hx as a Point2f
	Point2f* xPts = (Point2f*)hx;

	for (int i=0; i<2; i++)
	{
		vector<Point3f> gPts;
		for (int j=0; j<data->nglints[i]; j++)
		{
			Point3f cvec1 = normalize(data->camPos[i]-*cornea);
			Point3f cvec2 = normalize(data->ledPos[j]-*cornea);
			cvec2 += cvec1;
			Point3f rpos = normalize(cvec2)*vpConfig::corneaRadius;
			rpos = rpos+(*cornea);

			gPts.push_back(Point3f(rpos));
		}
		vector<Point2f> gProj;
		data->cam->project(gPts, (i==0), gProj);

		// set our computed data array
		for (int j=0; j<data->nglints[i]; j++)
		{
			*xPts = gProj[j];
			xPts++;
		}
	}
}


// Go through and iteratively find the approximate cornea center
float vpEye3::findCorneaCenter()
{
	CorneaData data;

	// for projecting and unprojecting, set static var for the static function
	data.cam = vpConfig::getCamera();
	data.camPos[0] = Point3f();
	data.camPos[1] = data.cam->RPos();
	data.ledPos[0] = vpConfig::ledPos[0];
	data.ledPos[1] = vpConfig::ledPos[1];
	
	if (this->eyes2[0]->glints.size() < 2 || this->eyes2[1]->glints.size() < 2)
		return -1;

	// our glint x-y coordinates
	// two coords per LED per view max
	Point2f glints[2*N_LED];
	// so we can index it as a float array as well as Point2fs
	float* x = &glints[0].x;

	// arrays levmar requests
	LevmarInfo info;
	float* infof = &info.e2i;
	float work[LM_DIF_WORKSZ(3,4*N_LED)];

	int maxiter = 200;

	int size0 = this->eyes2[0]->glints.size();
	int size1 = this->eyes2[1]->glints.size();

	float bestres = 1e10;
	Point3f bestpos;

	// arbitrary, some close I.C.
	Point3f corneaPos(0, 0, 0.5f);

	// total number of tests
	int ntest = 0;

	LevmarOpts opts;

	opts.tau *= 10;

	// sort both original glint arrays by x-coordinate
	for (int i=0; i<2; i++)
		sort(this->eyes2[0]->glints.begin(),this->eyes2[0]->glints.end(),pt2sort);
	
	// now use bit-twiddling to enumerate all subsets
	for (int i=0; i<(1<<size0); i++)
	{
		data.nglints[0] = bitcount(i);
		// skip if too many or too few bits set
		if (data.nglints[0] > N_LED || data.nglints[0] == 0)
			continue;

		// now make an array of the points, and sort
		vector<Point2f> pts0;
		for (int k=0; k<size0; k++)
		{
			// include this point?
			if (i & (1<<k))
				pts0.push_back(this->eyes2[0]->glints[k]);
		}
		// and set x points array
		Point2f* xset0 = glints;
		for (int k=0; k<pts0.size(); k++)
		{
			*xset0 = pts0[k];
			xset0++;
		}
		
		// now loop through all possible 2nd thingy subsets
		for (int j=0; j<(1<<size1); j++)
		{
			data.nglints[1] = bitcount(j);
			// again, skip if too many or too few bits set
			if (data.nglints[1] > N_LED || data.nglints[1] == 0)
				continue;

			// now make an array of the points, and sort
			vector<Point2f> pts1;
			for (int k=0; k<size1; k++)
			{
				// include this point?
				if (j & (1<<k))
					pts1.push_back(this->eyes2[1]->glints[k]);
			}
			
			// now set rest of points array
			Point2f* xset1 = xset0;
			for (int k=0; k<pts1.size(); k++)
			{
				*xset1 = pts1[k];
				xset1++;
			}

			int ntot = 2*(data.nglints[0] + data.nglints[1]);

			// now get the results, yo
			slevmar_dif( &reprojectError,
				&corneaPos.x, x, 3, ntot, maxiter, (float*)&opts,
				infof, work, NULL, &data);
			ntest++;

			// and check if it's better, using min-sq errorz
			float res = (1.0f+sqrt(2*info.e2f/ntot))/ntot/ntot;
			if (res < bestres)
			{
				bestres = res;
				bestpos = corneaPos;
				this->corneaGlints[0] = pts0;
				this->corneaGlints[1] = pts1;
			}
		}
	}

	// now loop through all of the possible glint permutations

	cout << ntest << ", " << bestres << endl;

	corneaCenter = bestpos;
	this->lastData = data;

	// compute 2d cornea centers for each eye
	vector<Point3f> pts;
	pts.push_back(this->corneaCenter);
	for (int i=0; i<2; i++)
	{
		vector<Point2f> outpts;
		data.cam->project(pts, (i==0), outpts);
		this->corneaPos2[i] = outpts[0];
	}

	return info.e2f;
}


// Draw the center of the cornea on each eye's debug image
void vpEye3::drawCorneaCenter()
{
	vpStereoCamera *cam = vpConfig::getCamera();

	// use precalculated 2d cornea centers
	for (int i=0; i<2; i++)
	{
		// convert the coordinate to dbg display and draw green +
		Point2f pt = VP_EYE_DBG_SCL*(this->corneaPos2[i] - this->eyes2[i]->_offsetf);
		Point p = Point(pt);
		cv::line(this->eyes2[i]->imgDebug, p-Point(0,10), p+Point(0,10), CV_RGB(0,255,0));
		cv::line(this->eyes2[i]->imgDebug, p-Point(10,0), p+Point(10,0), CV_RGB(0,255,0));

		// draw each glint used in localization
		for (int j=0; j<this->corneaGlints[i].size(); j++)
		{
			p = Point(VP_EYE_DBG_SCL*(this->corneaGlints[i][j] - this->eyes2[i]->_offsetf));
			cv::circle(this->eyes2[i]->imgDebug, p, 8, CV_RGB(0,255,255));
		}

		// draw reflections of LEDs, if we can
		if (fabs(this->lastData.ledPos[0].x) < 0.0001f)
			return;
		vector<Point3f> gPts;
		for (int j=0; j<N_LED; j++)
		{
			Point3f cvec1 = normalize(lastData.camPos[i]-corneaCenter);
			Point3f cvec2 = normalize(lastData.ledPos[j]-corneaCenter);
			cvec2 += cvec1;
			Point3f rpos = normalize(cvec2)*vpConfig::corneaRadius;
			rpos = rpos+corneaCenter;

			gPts.push_back(Point3f(rpos));
		}
		vector<Point2f> gProj;
		lastData.cam->project(gPts, (i==0), gProj);

		// set our computed data array
		for (int j=0; j<N_LED; j++)
		{
			p = Point(VP_EYE_DBG_SCL*(gProj[j] - this->eyes2[i]->_offsetf));
			cv::line(this->eyes2[i]->imgDebug, p-Point(0,10), p+Point(0,10), CV_RGB(255,0,255));
			cv::line(this->eyes2[i]->imgDebug, p-Point(10,0), p+Point(10,0), CV_RGB(255,0,255));
		}
	}
}