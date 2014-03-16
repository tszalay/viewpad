/* Code based on OpenCV's StarDetect algorithm */
#include "stdafx.h"

#include "vpPointDetector.h"

//{{2, 1}, {4, 2}, {6, 3}, {8, 4}, {12, 6}, {16, 8}, {22, 11}, {32, 16}, {46, 23},
 //{64, 32}, {90, 45}, {128, 64}, }

const int vpPointDetector::sizes0[MAX_PATTERN] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 22, 24, 32, 44, -1};
const int vpPointDetector::pairs[N_PAIRS][2] = {{1, 0}, {3, 1}, {5, 2}, {7, 3}, {9, 4}, {11, 5}, {12, 7}, {13, 9}, {13, 10},
									{14, 11}, {15, 12}, {16, 13}, {-1, -1}};


// allocate all matrices, and create the feature shapes
vpPointDetector::vpPointDetector(int width, int height) : rows(height), cols(width)
{
	mSum = cvCreateMat( rows + 1, cols + 1, CV_32SC1 );
    mTilted = cvCreateMat( rows + 1, cols + 1, CV_32SC1 );
    mFlatTilted = cvCreateMat( rows + 1, cols + 1, CV_32SC1 );

	mResponses = cvCreateMat( rows, cols, CV_32FC1 );
    mSizes = cvCreateMat( rows, cols, CV_16SC1 );

	// set default params
	maxSize = 7;
	suppressNonmaxSize = 6;
	responseThreshold = 18;
	lineThresholdProjected = 10;
	lineThresholdBinarized = 8;
	checkMin = false;
	checkMax = true;

	// create the feature pointers - all of them
	int step = mSum->step/CV_ELEM_SIZE(mSum->type);

    for(int i = 0; i < MAX_PATTERN; i++ )
    {
        int ur_size = sizes0[i];
		int t_size = sizes0[i] + sizes0[i]/2;
        int ur_area = (2*ur_size + 1)*(2*ur_size + 1);
        int t_area = t_size*t_size + (t_size + 1)*(t_size + 1);

		features[i] = vpPointFeature();

        features[i].p[0] = mSum->data.i + (ur_size + 1)*step + ur_size + 1;
        features[i].p[1] = mSum->data.i - ur_size*step + ur_size + 1;
        features[i].p[2] = mSum->data.i + (ur_size + 1)*step - ur_size;
        features[i].p[3] = mSum->data.i - ur_size*step - ur_size;

        features[i].p[4] = mTilted->data.i + (t_size + 1)*step + 1;
        features[i].p[5] = mFlatTilted->data.i - t_size;
        features[i].p[6] = mFlatTilted->data.i + t_size + 1;
        features[i].p[7] = mTilted->data.i - t_size*step + 1;

        features[i].area = ur_area + t_area;
        sizes1[i] = sizes0[i];
    }

	// compute their inverse weights
    for( int i = 0; i < N_PAIRS; i++ )
    {
        int innerArea = features[pairs[i][1]].area;
        int outerArea = features[pairs[i][0]].area - innerArea;
        invSizes[i][0] = 1.f/outerArea;
        invSizes[i][1] = 1.f/innerArea;
    }
}

vpPointDetector::~vpPointDetector()
{
	cvReleaseMat(&mSum);
	cvReleaseMat(&mTilted);
	cvReleaseMat(&mFlatTilted);
	cvReleaseMat(&mResponses);
	cvReleaseMat(&mSizes);
}

void vpPointDetector::computeIntegralImages( const CvMat* matI )
{
    int x, y;

	// pointer to the source image data
    const uchar* I = matI->data.ptr;

	// pointers to the sum, tilted sum, and flat tilted images
	int *S = this->mSum->data.i, *T = this->mTilted->data.i, *FT = this->mFlatTilted->data.i;
    int istep = matI->step, step = this->mSum->step/sizeof(S[0]);
    
	// check to make sure our datatypes are ok
    assert( CV_MAT_TYPE(matI->type) == CV_8UC1);

	// set first row to 0
    for( x = 0; x <= cols; x++ )
        S[x] = T[x] = FT[x] = 0;

	// now set second row
    S += step; T += step; FT += step;
    S[0] = T[0] = 0;
    FT[0] = I[0];
    for( x = 1; x < cols; x++ )
    {
        S[x] = S[x-1] + I[x-1];
        T[x] = I[x-1];
        FT[x] = I[x] + I[x-1];
    }
    S[cols] = S[cols-1] + I[cols-1];
    T[cols] = FT[cols] = I[cols-1];

    for( y = 2; y <= rows; y++ )
    {
        I += istep, S += step, T += step, FT += step;

		// set the beginnings of current row
        S[0] = S[-step]; S[1] = S[-step+1] + I[0];
        T[0] = T[-step + 1];
        T[1] = FT[0] = T[-step + 2] + I[-istep] + I[0];
        FT[1] = FT[-step + 2] + I[-istep] + I[1] + I[0];
		
		// and set the row
        for( x = 2; x < cols; x++ )
        {
            S[x] = S[x - 1] + S[-step + x] - S[-step + x - 1] + I[x - 1];
            T[x] = T[-step + x - 1] + T[-step + x + 1] - T[-step*2 + x] + I[-istep + x - 1] + I[x - 1];
            FT[x] = FT[-step + x - 1] + FT[-step + x + 1] - FT[-step*2 + x] + I[x] + I[x-1];
        }

        S[cols] = S[cols - 1] + S[-step + cols] - S[-step + cols - 1] + I[cols - 1];
        T[cols] = FT[cols] = T[-step + cols - 1] + I[-istep + cols - 1] + I[cols - 1];
    }
}


// fill in response and size matrices
int vpPointDetector::computeResponses( const CvMat* img )
{
	// check how many sizes we need to consider
	int i = 0;
    while( pairs[i][0] >= 0 && !
          ( sizes0[pairs[i][0]] >= this->maxSize 
           || sizes0[pairs[i+1][0]] + sizes0[pairs[i+1][0]]/2 >= min(rows, cols) ) )
    {
        ++i;
    }
    
    int npatterns = i;
	npatterns += (pairs[npatterns-1][0] >= 0);
    int maxIdx = pairs[npatterns-1][0];

    int border = sizes0[maxIdx] + sizes0[maxIdx]/2;
	int step = mSum->step/CV_ELEM_SIZE(mSum->type);    

	// zero response/size arrays
    for(int y = 0; y < border; y++ )
    {
        float* r_ptr = (float*)(mResponses->data.ptr + mResponses->step*y);
        float* r_ptr2 = (float*)(mResponses->data.ptr + mResponses->step*(rows - 1 - y));
        short* s_ptr = (short*)(mSizes->data.ptr + mSizes->step*y);
        short* s_ptr2 = (short*)(mSizes->data.ptr + mSizes->step*(rows - 1 - y));
        
        memset( r_ptr, 0, cols*sizeof(r_ptr[0]));
        memset( r_ptr2, 0, cols*sizeof(r_ptr2[0]));
        memset( s_ptr, 0, cols*sizeof(s_ptr[0]));
        memset( s_ptr2, 0, cols*sizeof(s_ptr2[0]));
    }

	// compute maximal size responses
    for( int y = border; y < rows - border; y++ )
    {
        int x = border, i;
        float* r_ptr = (float*)(mResponses->data.ptr + mResponses->step*y);
        short* s_ptr = (short*)(mSizes->data.ptr + mSizes->step*y);
        
        memset( r_ptr, 0, border*sizeof(r_ptr[0]));
        memset( s_ptr, 0, border*sizeof(s_ptr[0]));
        memset( r_ptr + cols - border, 0, border*sizeof(r_ptr[0]));
        memset( s_ptr + cols - border, 0, border*sizeof(s_ptr[0]));
  
        for( ; x < cols - border; x++ )
        {
            int ofs = y*step + x;
            int vals[MAX_PATTERN];
            float bestResponse = 0;
            int bestSize = 0;

			// compute responses to each star shape
            for( i = 0; i <= maxIdx; i++ )
            {
                const int** p = (const int**)&features[i].p[0];
                vals[i] = p[0][ofs] - p[1][ofs] - p[2][ofs] + p[3][ofs] +
                    p[4][ofs] - p[5][ofs] - p[6][ofs] + p[7][ofs];
            }
			// use inner/outer pairs to compute response to each pattern
            for( i = 0; i < npatterns; i++ )
            {
                int inner_sum = vals[pairs[i][1]];
                int outer_sum = vals[pairs[i][0]] - inner_sum;
                float response = inner_sum*invSizes[i][1] - outer_sum*invSizes[i][0];
                if( fabs(response) > fabs(bestResponse) )
                {
                    bestResponse = response;
                    bestSize = sizes1[pairs[i][0]];
                }
            }

			// fill in best response and best size
            r_ptr[x] = bestResponse;
            s_ptr[x] = (short)bestSize;
        }
    }

    return border;
}


// returns true if lines are found
bool vpPointDetector::suppressLines( CvPoint pt )
{
    const float* r_ptr = mResponses->data.fl;
    int rstep = mResponses->step/sizeof(r_ptr[0]);
    const short* s_ptr = mSizes->data.s;
    int sstep = mSizes->step/sizeof(s_ptr[0]);
    int sz = s_ptr[pt.y*sstep + pt.x];

	// quit if delta would otherwise be 0
	if (abs(sz) < 4) return false;

    int x, y, delta = sz/4, radius = delta*4;
    float Lxx = 0, Lyy = 0, Lxy = 0;
    int Lxxb = 0, Lyyb = 0, Lxyb = 0;
    
    for( y = pt.y - radius; y <= pt.y + radius; y += delta )
        for( x = pt.x - radius; x <= pt.x + radius; x += delta )
        {
            float Lx = r_ptr[y*rstep + x + 1] - r_ptr[y*rstep + x - 1];
            float Ly = r_ptr[(y+1)*rstep + x] - r_ptr[(y-1)*rstep + x];
            Lxx += Lx*Lx; Lyy += Ly*Ly; Lxy += Lx*Ly;
        }
    
    if( (Lxx + Lyy)*(Lxx + Lyy) >= this->lineThresholdProjected*(Lxx*Lyy - Lxy*Lxy) )
        return true;

    for( y = pt.y - radius; y <= pt.y + radius; y += delta )
        for( x = pt.x - radius; x <= pt.x + radius; x += delta )
        {
            int Lxb = (s_ptr[y*sstep + x + 1] == sz) - (s_ptr[y*sstep + x - 1] == sz);
            int Lyb = (s_ptr[(y+1)*sstep + x] == sz) - (s_ptr[(y-1)*sstep + x] == sz);
            Lxxb += Lxb * Lxb; Lyyb += Lyb * Lyb; Lxyb += Lxb * Lyb;
        }

    if( (Lxxb + Lyyb)*(Lxxb + Lyyb) >= this->lineThresholdBinarized*(Lxxb*Lyyb - Lxyb*Lxyb) )
        return true;

    return false;
}

// only add point if it is the highest response in the nonmax neighborhood
void vpPointDetector::suppressNonmax( vector<CvStarKeypoint>& keypoints, int border )
{
    int x, y, x1, y1;
	int delta = this->suppressNonmaxSize/2;
    const float* r_ptr = mResponses->data.fl;
    int rstep = mResponses->step/sizeof(r_ptr[0]);
    const short* s_ptr = mSizes->data.s;
    int sstep = mSizes->step/sizeof(s_ptr[0]);
    short featureSize = 0;

	// scan responses tile-by-tile
    for( y = border; y < rows - border; y += delta+1 )
        for( x = border; x < cols - border; x += delta+1 )
        {
            float maxResponse = this->responseThreshold;
            float minResponse = -this->responseThreshold;
            CvPoint maxPt = {-1,-1}, minPt = {-1,-1};
            int tileEndY = MIN(y + delta, rows - border - 1);
            int tileEndX = MIN(x + delta, cols - border - 1);

			// check neighborhood, find largest response in area
            for( y1 = y; y1 <= tileEndY; y1++ )
                for( x1 = x; x1 <= tileEndX; x1++ )
                {
                    float val = r_ptr[y1*rstep + x1];
                    if( maxResponse < val )
                    {
                        maxResponse = val;
                        maxPt = cvPoint(x1, y1);
                    }
                    else if( minResponse > val )
                    {
                        minResponse = val;
                        minPt = cvPoint(x1, y1);
                    }
                }

			// if we found something
            if( maxPt.x >= 0 && this->checkMax)
            {
				// check around it, see if it is actually the largest point in the neighborhood
                for( y1 = maxPt.y - delta; y1 <= maxPt.y + delta; y1++ )
                    for( x1 = maxPt.x - delta; x1 <= maxPt.x + delta; x1++ )
                    {
                        float val = r_ptr[y1*rstep + x1];
                        if( val >= maxResponse && (y1 != maxPt.y || x1 != maxPt.x))
                            goto skip_max; // it's not; we'll add the largest one later
                    }

				// now add it, if we deem it worthy
                if( (featureSize = s_ptr[maxPt.y*sstep + maxPt.x]) >= 0 &&
                    !suppressLines( maxPt ))
                {
                    CvStarKeypoint kpt = cvStarKeypoint( maxPt, featureSize, maxResponse );
                    keypoints.push_back(kpt);
                }
            }
skip_max:
			// only search for minima if we are supposed to
            if( minPt.x >= 0 && this->checkMin)
            {
                for( y1 = minPt.y - delta; y1 <= minPt.y + delta; y1++ )
                    for( x1 = minPt.x - delta; x1 <= minPt.x + delta; x1++ )
                    {
                        float val = r_ptr[y1*rstep + x1];
                        if( val <= minResponse && (y1 != minPt.y || x1 != minPt.x))
                            goto skip_min;
                    }

                if( (featureSize = s_ptr[minPt.y*sstep + minPt.x]) >= 0 &&
                    !suppressLines( minPt ))
                {
                    CvStarKeypoint kpt = cvStarKeypoint( minPt, featureSize, minResponse );
                    keypoints.push_back(kpt);
                }
            }
skip_min:
            ;
        }
}

vector<CvStarKeypoint> vpPointDetector::getKeypoints( const IplImage* _img )
{
    CvMat stub, *img = cvGetMat(_img, &stub);
    vector<CvStarKeypoint> keypoints;

	computeIntegralImages( img );
    int border = computeResponses( img );
	suppressNonmax( keypoints, border );

    return keypoints;
}