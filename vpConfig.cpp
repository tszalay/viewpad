#include "stdafx.h"
#include <fstream>

vpStereoCamera* vpConfig::_cam = 0;
const float vpConfig::corneaRadius = 0.008f;

int64 vpConfig::lastTick = cv::getTickCount();
double vpConfig::tickFreq = 1;

Point3f vpConfig::screenPos[4];
Point3f vpConfig::ledPos[N_LED];

void vpConfig::loadTargets()
{
	FileStorage fs(VP_CALIB_FILE, FileStorage::READ);

	Mat tgtPts;
	fs["tgtPts"] >> tgtPts;

	if (!tgtPts.data)
	{
		cout << "No target points." << endl;
		return;
	}

	for (int i=0; i<4; i++)
	{
		screenPos[i].x = tgtPts.at<float>(i,0);
		screenPos[i].y = tgtPts.at<float>(i,1);
		screenPos[i].z = tgtPts.at<float>(i,2);
	}
	for (int i=4; i<N_TGT; i++)
	{
		ledPos[i-4].x = tgtPts.at<float>(i,0);
		ledPos[i-4].y = tgtPts.at<float>(i,1);
		ledPos[i-4].z = tgtPts.at<float>(i,2);
	}

	cout << "Target points loaded." << endl;
}

void vpConfig::tick(char* str)
{
	int64 dtick = cv::getTickCount()-lastTick;
	double dt = 1000.0*(dtick/tickFreq); // in ms
	cout << str << " time: " << dt << endl;
	lastTick = cv::getTickCount();
}


void vpConfig::init()
{
	vpConfig::loadTargets();

	vpConfig::tickFreq = cv::getTickFrequency();

	ofstream dbg;
	dbg.open(VP_DBG_FILE, ios::out);
	dbg << "dbg = {Red};" << endl;
	dbg.close();
	// and draw axes, why not
	writeLn(Point3f(),Point3f(0.5f,0,0));
	writeLn(Point3f(),Point3f(0,0.5f,0));
	writeLn(Point3f(),Point3f(0,0,0.5f));
	writeStr("Black");
}

void vpConfig::close()
{
	ofstream dbg;
	dbg.open(VP_DBG_FILE, ios::app | ios::out);
	dbg << "Graphics3D[dbg]" << endl;
	dbg.close();
}

void vpConfig::writePt(Point3f pt)
{
	ofstream dbg;
	dbg.open(VP_DBG_FILE, ios::app | ios::out);
	dbg << "dbg=Append[dbg,Sphere[";
	dbg << "{" << pt.x << "," << pt.y << "," << pt.z << "}, 0.01]];" << endl;
	dbg.close();
}

void vpConfig::writeLn(Point3f pt1, Point3f pt2)
{
	ofstream dbg;
	dbg.open(VP_DBG_FILE, ios::app | ios::out);
	dbg << "dbg=Append[dbg,Line[{";
	dbg << "{" << pt1.x << "," << pt1.y << "," << pt1.z << "},";
	dbg << "{" << pt2.x << "," << pt2.y << "," << pt2.z << "}}]];" << endl;
	dbg.close();
}

void vpConfig::writeStr(const char * str)
{
	ofstream dbg;
	dbg.open(VP_DBG_FILE, ios::app | ios::out);
	dbg << "dbg=Append[dbg," << str << "];" << endl;
	dbg.close();
}

void vpConfig::writeLn(vector<Point3f> pts)
{
	ofstream dbg;
	dbg.open(VP_DBG_FILE, ios::app | ios::out);
	dbg << "dbg=Append[dbg,Line[{";
	for (int i=0; i<pts.size()-1; i++)
		dbg << "{" << pts[i].x << "," << pts[i].y << "," << pts[i].z << "},";
	dbg << "{" << pts.back().x << "," << pts.back().y << "," << pts.back().z << "}}]];" << endl;
	dbg.close();
}


Point3f normalize(Point3f vec)
{
	return vec*(1.0/sqrt(vec.dot(vec)));
}

Point3f coordinateTransform(Point3f pt, Mat& rot, Mat& trans)
{
	Mat ptM = Mat(3, 1, CV_32FC1, &pt.x);

	ptM = rot*ptM;
	ptM += trans;
	
	return pt;
}

Point3f unCoordinateTransform(Point3f pt, Mat& rot, Mat& trans)
{
	Mat ptM = Mat(3, 1, CV_32FC1, &pt.x);

	ptM -= trans;
	ptM = rot.t()*ptM;
	
	return pt;
}