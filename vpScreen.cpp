#include "stdafx.h"
#include "vpProcess.h"


void vpEye3::findScreenPoint()
{
	Point3f eyeDir = normalize(this->pupilCenter-this->corneaCenter);

	// a point on the screen
	Point3f pt0 = vpConfig::screenPos[0];
	Point3f pt1 = vpConfig::screenPos[1] - pt0;
	Point3f pt2 = vpConfig::screenPos[3] - pt0;
	// screen normal vector
	Point3f screenNormal = normalize(pt1.cross(pt2));

	// distance along ray
	float lined = -(screenNormal.dot(this->corneaCenter-pt0))/screenNormal.dot(eyeDir);
	// intersection point
	Point3f ptint = this->corneaCenter + lined*eyeDir;

	float xdist = (ptint - pt0).dot(pt1);
	float ydist = (ptint - pt0).dot(pt2);

	xdist = xdist / sqrt(pt1.dot(pt1));
	ydist = ydist / sqrt(pt2.dot(pt2));

	this->screenPos = Point2f(xdist*1920,ydist*1080);

	int xcoord = (int)(xdist*1920);
	int ycoord = (int)(ydist*1080);

	cout << "Pos: (" << xcoord << ", " << ycoord << ")" << endl;
}