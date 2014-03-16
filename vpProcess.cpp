#include "stdafx.h"
#include "vpProcess.h"



bool vpProcess::init()
{
	haarLeft = (CvHaarClassifierCascade*)cvLoad(VP_EYEHAARL, 0, 0, 0);
	haarRight = (CvHaarClassifierCascade*)cvLoad(VP_EYEHAARR, 0, 0, 0);
	haarBoth = (CvHaarClassifierCascade*)cvLoad(VP_EYEHAARB, 0, 0, 0);

	storage = cvCreateMemStorage(0);

	return (haarLeft != NULL && haarRight != NULL && haarBoth != NULL);
}