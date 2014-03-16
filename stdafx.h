// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#define _ITERATOR_DEBUG_LEVEL 0
#define M_PI 3.1415926

#include "targetver.h"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv/highgui.h"
#include "videoInput.h"

#include <stdio.h>
#include <tchar.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h> 

#include <iostream>
#include <vector>

#include <WinUser.h>

#include "vpStereoCamera.h"
#include "levmar-2.5/levmar.h"
#include "vpConfig.h"


// handy-dandy normalize function
Point3f normalize(Point3f vec);
// transformation function for rotation/translation
Point3f coordinateTransform(Point3f pt, Mat& rot, Mat& trans);
Point3f unCoordinateTransform(Point3f pt, Mat& rot, Mat& trans);