#include "stdafx.h"

#ifndef __VPPOINTDETECTOR_H__
#define __VPPOINTDETECTOR_H__

using namespace std;
using namespace cv;

class vpPointDetector
{
public:

	vpPointDetector(int width, int height);
	~vpPointDetector();

	// find all of the keypoints in an image
	vector<CvStarKeypoint> vpPointDetector::getKeypoints( const IplImage* _img );

	// global settings
	int maxSize;
	int suppressNonmaxSize;
	float responseThreshold;
	float lineThresholdProjected;
	float lineThresholdBinarized;
	bool checkMin;
	bool checkMax;

private:
	void vpPointDetector::computeIntegralImages( const CvMat* matI );
	bool vpPointDetector::suppressLines( CvPoint pt );
	void vpPointDetector::suppressNonmax( vector<CvStarKeypoint>& keypoints, int border );
	int vpPointDetector::computeResponses( const CvMat* img );

	// describes a star shape, and associated pointers in an image
	struct vpPointFeature
	{
		int area;
		int* p[8];
	};


	// INTERNAL VARIABLES

	// maximum number of patterns to consider
	static const int MAX_PATTERN = 18;
	static const int N_PAIRS = 13;
	
	// array of sizes, to be indexed by the pairs array
	static const int sizes0[MAX_PATTERN];
	// indices into the sizes0 array, denoting outer and inner sizes
	static const int pairs[N_PAIRS][2];

	// global sizes etc
	const int rows;
	const int cols;

	// inverse areas of inner/outer regions
    float invSizes[N_PAIRS][2];
    int sizes1[MAX_PATTERN];

	// structs to hold pointers for each of our features of all sizes
    vpPointFeature features[MAX_PATTERN];

	// integral data
    CvMat *mSum, *mTilted, *mFlatTilted;

	// response data
	CvMat* mResponses;
    CvMat* mSizes;
};

#endif