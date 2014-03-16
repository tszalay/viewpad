#include "stdafx.h"
#include "vpCalib.h"
#include <fstream>

using namespace std;
using namespace cv;


// these functions and structs are just internal ones for use with LM solver for chessboard positions

// this is the function called by Levenberg-Marquardt, to minimize error
// p is the array of parameters, or mirrordata
// hx is output screen coord xy of projected points, total length n
void projectMirror(float *p, float *hx, int m, int n, void *adata)
{
	MirrorData* mdata = (MirrorData*)p;

	Point3f vX;
	Point3f vY;

	mdata->getVectors(vX, vY);

	// compute all of the chessboard points
	vector<Point3f> worldPts;
	for (int y=0; y<VP_CALIB_BHEIGHT; y++)
		for (int x=0; x<VP_CALIB_BWIDTH; x++)
			worldPts.push_back(x*vX + y*vY + mdata->orig);

	// unproject them as well, in both views
	vpStereoCamera* cam = (vpStereoCamera*)adata;

	vector<Point2f> outpts[2];
	for (int i=0; i<2; i++)
		cam->project(worldPts,i==0,outpts[i]);

	// and write all of our points
	Point2f* pt = (Point2f*)hx;
	for (int i=0; i<2; i++)
	{
		for (int j=0; j<VP_CALIB_BWIDTH*VP_CALIB_BHEIGHT; j++)
		{
			*pt = outpts[i][j];
			pt++;
		}
	}
}


bool vpCalib::doCalibrate(int nwidth, int nheight, float blockdim)
{
	int board_n = nwidth*nheight;
	CvSize board_sz = cvSize( nwidth, nheight );

	int numboards = (int)images[0].size();

	// growable storage for all of the points we're going to grab
	vector<vector<Point2f> > image_points_L;
	vector<vector<Point2f> > image_points_R;

	// find as many chessboards as we can between the two boards
	cout << "Beginning search with " << numboards << " images" << endl;

	int numfound = 0;

	vector<Mat> goodImages[2];

	for (int i=0; i<numboards; i++)
	{
		cout << endl << i << "... ";
		// Find chessboard corners:	
		vector<Point2f> corners_L;
		int found = cv::findChessboardCorners( images[0][i], board_sz, corners_L,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

		// skip if not all found
		if (corners_L.size() < board_n)
			continue;

		vector<Point2f> corners_R;
		found = cv::findChessboardCorners( images[1][i], board_sz, corners_R,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

		// skip if not all found
		if (corners_R.size() < board_n)
			continue;

		image_points_L.push_back(corners_L);
		image_points_R.push_back(corners_R);

		for (int j=0; j<2; j++)
			goodImages[j].push_back(images[j][i]);
		
		cout << "good.";

		numfound++;
	}

	cout << endl << "Found " << numfound << " good images." << endl;

	if (numfound < 3)
	{
		cout << "Found too few images, aborting." << endl;
		return false;
	}

	// the rest are the same for left and right
	vector<vector<Point3f> > object_points;
	
	
	// and the output camera matrices
	Mat intrinsic_matrix_L;
	Mat intrinsic_matrix_R;
	Mat distortion_matrix_L;
	Mat distortion_matrix_R;

	// rotation and translation between two cameras
	Mat rot_matrix;
	Mat trans_matrix;
	
	// Set the object points, with chessboard plane in x/y
	vector<Point3f> objPts;
	for (int i=0; i<board_n; i++)
		objPts.push_back(Point3f((i/nwidth)*blockdim,(i%nwidth)*blockdim,0));

	// and populate the main vector with lots of duplicates
	for( int i = 0; i < numfound; ++i )
		object_points.push_back(objPts);

	// At this point we have all the chessboard corners we need

	cout << "Attemping calibration... ";

	int flags = 0;//CV_CALIB_FIX_PRINCIPAL_POINT;
	double resL = cv::calibrateCamera( object_points, image_points_L, images[0][0].size(), intrinsic_matrix_L, distortion_matrix_L, mirrorRot[0], mirrorPos[0], flags);
	double resR = cv::calibrateCamera( object_points, image_points_R, images[0][0].size(), intrinsic_matrix_R, distortion_matrix_R, mirrorRot[1], mirrorPos[1], flags);

	Mat E;
	Mat F;

	double resS = cv::stereoCalibrate( object_points, image_points_L, image_points_R, intrinsic_matrix_L, distortion_matrix_L, 
		intrinsic_matrix_R, distortion_matrix_R, images[0][0].size(), rot_matrix, trans_matrix, 
		E, F, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 200, 1e-6), CV_CALIB_FIX_INTRINSIC);

	cout << "complete." << endl;
	
	cout << "Intrinsic calibration error = (" << resL << ", " << resR << ")" << endl;
	cout << "Stereo calibration error = " << resS << endl;

	cout << "Saving files to cam/ ..." << endl;

	// Save the intrinsics and distortions
	try {
		Mat tmp;
		FileStorage fs(VP_CALIB_FILE, FileStorage::WRITE);

		intrinsic_matrix_L.convertTo(tmp, CV_32FC1);
		fs << "intr_L" << tmp;

		intrinsic_matrix_R.convertTo(tmp, CV_32FC1);
		fs << "intr_R" << tmp;

		distortion_matrix_L.convertTo(tmp, CV_32FC1);
		fs << "dist_L" << tmp;
		distortion_matrix_R.convertTo(tmp, CV_32FC1);
		fs << "dist_R" << tmp;
		
		rot_matrix.convertTo(tmp, CV_32FC1);
		fs << "rot" << tmp;
		trans_matrix.convertTo(tmp, CV_32FC1);
		fs << "trans" << tmp;

		cout << "Save complete." << endl;
	}
	catch (...) {
		cout << "Save failed." << endl;
		return false;
	}

	// stop capturing, start post-capture analysis
	capturing = false;

	// now set image arrays to store only the good images
	for (int i=0; i<2; i++)
		images[i] = goodImages[i];

	int nimages = images[0].size();

	for (int i=0; i<2; i++)
	{
		targetPoints[i] = new Point*[nimages];
		for (int j=0; j<nimages; j++)
			targetPoints[i][j] = new Point[N_TGT];
	}

	// reload calib data
	cam->loadCalibration();

	// output some debug points, such as camera direction etc.
	Point3f pt(0,0,0.1f);
	vpConfig::writeLn(Point3f(),pt);
	vpConfig::writeLn(cam->RtoL(Point3f()),cam->RtoL(pt));

	// now compute the optimal positions of all of the chessboards in each view

	// array for holding coords of chessboards in current two views
	Point2f chessboards[2*VP_CALIB_BNUM];
	// so we can index it as a float array as well as Point2fs
	float* chessboardx = &chessboards[0].x;
	
	// arrays levmar requests
	LevmarInfo info;
	float* infof = &info.e2i;
	float work[LM_DIF_WORKSZ(6,4*VP_CALIB_BNUM)];
	int maxiter = 500;
	LevmarOpts opts;

	opts.tau *= 0.5f;
	opts.epsilon2 *= 0.01f;

	// loop through and run code
	for (int i=0; i<nimages; i++)
	{
		MirrorData mdata;
		// set chessboard point coordinates
		Point2f* ptr = chessboards;
		for (int j=0; j<2; j++)
		{
			for (int k=0; k<VP_CALIB_BNUM; k++)
			{
				*ptr = (j==0)?image_points_L[i][k]:image_points_R[i][k];
				ptr++;
			}
		}

		// initialize mirror position to view from left camera, at least for the origin
		// convert from vector to proper matrix form
		Mat mrot = Mat(1,3,CV_32FC1);
		Mat rot = Mat(3,3,CV_32FC1);
		mirrorRot[0][i].convertTo(mrot, CV_32FC1);
		cv::Rodrigues(mrot, rot);
		Mat trans = Mat(3,1,CV_32FC1);
		// transpose/convert by hand because opencv is fuxing useless
		for (int k=0; k<3; k++)
			trans.at<float>(k) = mirrorPos[0][i].at<double>(k);

		// get initial vX, vY
		Point3f ptBoard[3];
		ptBoard[0] = Point3f();
		ptBoard[1] = Point3f(VP_CALIB_BDIM,0,0);
		ptBoard[2] = Point3f(0,VP_CALIB_BDIM,0);
		for (int j=0; j<3; j++)
			ptBoard[j] = coordinateTransform(ptBoard[j], rot, trans);
		// get relative vecs
		ptBoard[1] = ptBoard[1] - ptBoard[0];
		ptBoard[2] = ptBoard[2] - ptBoard[0];
		// get normal vector
		Point3f norm = -normalize(ptBoard[1].cross(ptBoard[2]));
		// and convert it to spherical coords
		mdata.dir.x = acos(norm.z);
		mdata.dir.y = atan2(norm.y,norm.x);

		mdata.orig = ptBoard[0];


		// levmar wooooooooooooooooooooooooooooooooo
		slevmar_dif( &projectMirror,
			(float*)&mdata, chessboardx, 6, 4*VP_CALIB_BNUM, maxiter, (float*)&opts,
			infof, work, NULL, cam);
		// check the result, by saving in debug info
		Point3f vX, vY;
		mdata.getVectors(vX, vY);
		vX = vX*VP_CALIB_BWIDTH;
		vY = vY*VP_CALIB_BHEIGHT;
		vector<Point3f> pts;
		pts.push_back(mdata.orig);
		pts.push_back(mdata.orig+vX);
		pts.push_back(mdata.orig+vX+vY);
		pts.push_back(mdata.orig+vY);
		pts.push_back(mdata.orig);
		cout << "iter : " << info.numIter << ", term: " << info.term << ", rot: " << mdata.dir << endl;
		vpConfig::writeStr("Orange");
		vpConfig::writeLn(pts);

		// and save the mirror data
		this->mirrorData.push_back(mdata);
	}
	// start mouse callback
	cvSetMouseCallback(VP_CALIB_TITLE, onMouse, this);

	cout << "Camera calibration complete, begin target selection." << endl;

	return true;
}



void vpCalib::finishCalibrate()
{
	// points defining line segments, per camera and target
	vector<Point3f> x0s[2][N_TGT];
	vector<Point3f> x1s[2][N_TGT];

	// we need to compute the rays towards each target point, and then reflect them
	for (int i=0; i<2; i++)
	{
		for (int j=0; j<images[0].size(); j++)
		{
			// first, unproject the pointses into world space (in l-coords)
			vector<Point2f> pts;
			for (int k=0; k < N_TGT; k++)
				pts.push_back(Point2f(targetPoints[i][j][k]));
			vector<Point3f> outpts;
			cam->unproject(pts, i==0, outpts);

			// compute the reflected points
			// using new reflection algorithm!
			// and mirrors from levmar
			for (int k=0; k < N_TGT; k++)
			{
				Point3f ptA = (i==0)?Point3f():cam->RtoL(Point3f());
				// skip over this point if we didn't set it, make sure we skip both cams
				if (targetPoints[0][j][k] == Point() || targetPoints[1][j][k] == Point())
					continue;
				// now reflect and compute
				Point3f ptB = outpts[k];
				vpConfig::writeStr((i==0)?"Green":"Blue");
				vpConfig::writeLn(ptA, ptB);
				vpConfig::writeStr((i==0)?"Purple":"Pink");
				mirrorData[j].reflectRay(ptA, ptB);
				vpConfig::writeLn(ptA, ptB);

				// now add the ray
				x0s[i][k].push_back(ptA);
				x1s[i][k].push_back(ptB);
			}
		}
	}

	
	// write output points as a matrix of x,y,z coords
	Mat tgtPts = Mat(N_TGT, 3, CV_32FC1);

	// now let's go through and compute the position of each target
	for (int i=0; i<N_TGT; i++)
	{
		int npts = (int)x0s[0][i].size();
		Point3f ptAvg = Point3f();

		if (!npts)
		{
			// set out point to 0
			for (int j=0; j<3; j++)
				tgtPts.at<float>(i,j) = 0;
			continue;
		}

		for (int j=0; j<npts; j++)
		{
			// parametric lines of form x0 + t*u0
			Point3f u0 = normalize(x1s[0][i][j] - x0s[0][i][j]);
			Point3f u1 = normalize(x1s[1][i][j] - x0s[1][i][j]);
			// find the closest point between two lines
			Point3f x10 = x0s[1][i][j] - x0s[0][i][j];
			Point3f m = u1.cross(u0);
			float m2 = m.dot(m);
			Point3f r = x10.cross(m)*(1.0f/m2);
			float t0 = r.dot(u1);
			float t1 = r.dot(u0);
			// points of closest approach
			Point3f q0 = x0s[0][i][j] + t0*u0;
			Point3f q1 = x0s[1][i][j] + t1*u1;
			Point3f pt = 0.5f*(q0+q1);
			
			ptAvg += pt;

			float dist = fabs(x10.dot(m))/sqrt(m2);
		}
		ptAvg = ptAvg * (1.0f/npts);
		vpConfig::writePt(ptAvg);
		vpConfig::writeCmt<int>(npts);
		// set out point to our point
		tgtPts.at<float>(i,0) = ptAvg.x;
		tgtPts.at<float>(i,1) = ptAvg.y;
		tgtPts.at<float>(i,2) = ptAvg.z;
	}

	try {
		// append the found target points to the file
		FileStorage fs(VP_CALIB_FILE, FileStorage::APPEND);
		fs << "tgtPts" << tgtPts;
	}
	catch (...)
	{
		cout << "Problem writing file." << endl;
	}

	// write and copy output file
	vpConfig::close();
	ifstream f1(VP_DBG_FILE, fstream::binary);
	ofstream f2(VP_DBGCAL_FILE, fstream::trunc|fstream::binary);
	f2 << f1.rdbuf();

	// reload target file for use
	vpConfig::loadTargets();

	// clean up our shitty memory shitty C++ shit
	for (int i=0; i<images[0].size(); i++)
	{
		delete [] targetPoints[0][i];
		delete [] targetPoints[1][i];
	}
	delete [] targetPoints[0];
	delete [] targetPoints[1];
}
