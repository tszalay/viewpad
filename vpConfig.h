#include "stdafx.h"

#ifndef __VPCONFIG_H__
#define __VPCONFIG_H__

#define N_LED 2
#define N_TGT (N_LED+4)

// size of eye image
#define VP_SUBSIZE 128

// filenames of eye finding haar files
//#define VP_EYEHAARL "res/haarcascade_eye.xml"
//#define VP_EYEHAARR "res/haarcascade_eye.xml"

#define VP_EYEHAARL "res/ojoI.xml"
#define VP_EYEHAARR "res/ojoD.xml"
#define VP_EYEHAARB "res/parojosG.xml"

// minimum size for haar eyes
#define VP_EYEMIN 30

// max value of y/x for eye detection
#define VP_EYEMAXROT 0.4


// eye debug image scale-up factor
#define VP_EYE_DBG_SCL 2
// main debug image scale-down factor
#define VP_MAIN_DBG_SCL 4
// calib debug image scale-down factor
#define VP_CAL_DBG_SCL 4


// width, height, and square dim (m) of chessboard
#define VP_CALIB_BWIDTH		9
#define VP_CALIB_BHEIGHT	5
#define VP_CALIB_BNUM		(VP_CALIB_BWIDTH*VP_CALIB_BHEIGHT)
#define VP_CALIB_BDIM		0.015

#define VP_CALIB_TITLE "Calibration Images"

#define VP_DBG_FILE "debugout.nb"
#define VP_DBGCAL_FILE "calibout.nb"

#define VP_CALIB_FILE "cam/calib.yml"

class vpConfig
{
public:
	static const float corneaRadius; // meters

	static vpStereoCamera* getCamera() { return _cam; }
	static void setCamera(vpStereoCamera* camera) { _cam = camera; }

	static void init();
	static void close();
	static void writePt(Point3f pt);
	static void writeLn(Point3f pt1, Point3f pt2);
	static void writeLn(vector<Point3f> pts);
	static void writeStr(const char * str);
	template <typename T> static void writeCmt(T t)
	{
		ofstream dbg;
		dbg.open(VP_DBG_FILE, ios::app | ios::out);
		dbg << "(* " << t << " *)" << endl;
		dbg.close();
	}

	static void loadTargets();

	static Point3f screenPos[4];
	static Point3f ledPos[N_LED];

	static void tick(char* str);

private:
	static vpStereoCamera* _cam;

	// last tick and frequency
	static int64 lastTick;
	static double tickFreq;
};


// the levmar info struct containing lots of floats
struct LevmarInfo
{
	float e2i;		// e^2 at initial p
	float e2f;		// e^2 at final p
	float Jte2;		// J^t error magnitude or something
	float Dp2;		// Dp^2 at final p
	float mumaxJtl;	// some other parameter or something
	float numIter;	// number of iterations done
	float term;		// why terminate: 1 - small gradient, 2 - small Dp, 3 - itmax
					// 4 - singular matrix, 5 - no further error red. possible
					// 6 - small e^2, 7 - invalid (NaN, Inf) values
	float numEval;	// total function evaluations
	float numJac;	// total number of jacobian evals
	float numLin;	// number of linear systems solved
};

// the levmar options struct
struct LevmarOpts
{
	float tau;			// scale factor for initial mu
	float epsilon1;		// stopping threshold for Jt
	float epsilon2;		// ditto for Dp^2
	float epsilon3;		// ditto for e^2
	float delta;		// difference approx. step

	LevmarOpts()
	{
		tau = LM_INIT_MU;
		epsilon1 = epsilon2 = epsilon3 = LM_STOP_THRESH;
		delta = LM_DIFF_DELTA;
	}
};


#endif