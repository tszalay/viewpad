/***************************
 *** POINT TRACKER CLASS ***
 ***************************/

// Created by Tamas Szalay
// May 23, 2010

#include "stdafx.h"
#include "vpTracker.h"
#include <algorithm>


// the status value new points start with
#define INIT_STATUS 1

// maximum status they can have
#define MAX_STATUS 10

// amount to increment by if successful track
#define INC_STATUS 2


// kalman filter properties, in pixels
#define KF_MEAS_NOISE 5e0
#define KF_PROC_NOISE 1e-0

// maximum association distance
#define MAX_ASSOC_DIST 30


// internal struct for association data, used to sort by closest
struct assocpt {
    int track;
    int meas;
    float dist;
};
bool assoc_comp(assocpt a, assocpt b) { return a.dist < b.dist; }

bool trackpts_comp(VPTrackPt a, VPTrackPt b) { return a.nassoc > b.nassoc; }


// predict, update, associate all points
void VPTracker::Update(vector<CvPoint2D32f> pts, float dt)
{
    CvMat* measurement = cvCreateMat(2, 1, CV_32FC1);

    // first, make a prediction for all of the points
    for (int i=0; i<(int)tracks.size(); i++)
    {
		tracks[i].dt += dt;
		// update state transition matrix with total dt since last assoc
		tracks[i].kalman->transition_matrix->data.fl[2] = tracks[i].dt;
		tracks[i].kalman->transition_matrix->data.fl[7] = tracks[i].dt;
		// and predict, gets placed into ptKalman->state_pre
		cvKalmanPredict(tracks[i].kalman);
    }

    // now we need to associate, create struct and fn
    vector<assocpt> assoc;

    // generate full n^2 list of possible assoc's
    for (int i=0; i<(int)pts.size(); i++)
    {
		for (int j=0; j<(int)tracks.size(); j++)
		{
			assocpt ass;
			// get distances
			float dx = pts[i].x - tracks[j].kalman->state_pre->data.fl[0];
			float dy = pts[i].y - tracks[j].kalman->state_pre->data.fl[1];
			ass.dist = sqrt(dx*dx+dy*dy);
	    
			if (ass.dist > MAX_ASSOC_DIST)
			continue;

			// add it to vector of potential associations
			ass.track = j;
			ass.meas = i;
			assoc.push_back(ass);
		}
    }

    // now sort grand list of associated distances
    sort(assoc.begin(), assoc.end(), assoc_comp);

    // create a list that tells us whether we have associated input pts
    int* assin = new int[pts.size()];

    for (int i=0; i<(int)pts.size(); i++)
		assin[i] = 0;

    // loop til list is empty
    for (vector<assocpt>::iterator ass=assoc.begin(); ass<assoc.end(); ass++)
    {	
		// we've already associated this measurement
		// or we've already associated with this track
		if (assin[ass->meas] > 0 || ass->track == -1)
			continue;
	
		// if it's good, we associate
		measurement->data.fl[0] = pts[ass->meas].x;
		measurement->data.fl[1] = pts[ass->meas].y;
		cvKalmanCorrect(tracks[ass->track].kalman, measurement);

		// increase status
		tracks[ass->track].status += INC_STATUS;
		tracks[ass->track].nassoc++;

		// reset dt
		tracks[ass->track].dt = 0;

		// say we did
		assin[ass->meas] = 1;

		// now remove this guy from the rest of the assoc array
		for (vector<assocpt>::iterator ass2=ass+1; ass2 < assoc.end(); ass2++)
			if (ass2->track == ass->track)
			ass2->track = -1;
		}

		// update all statuses
		for (int i=0; i<(int)tracks.size(); i++)
		{
		// decrement all	
		if (tracks[i].status > 0)
			tracks[i].status--;
		if (tracks[i].status > MAX_STATUS)
			tracks[i].status = MAX_STATUS;
	
		// if ready to remove, then remove, but stay on same index
		if (tracks[i].status == 0)
		{
			removePt(i);
			i--;
		}
    }

    // and create new points for unassociated inputs
    for (int i=0; i<(int)pts.size(); i++)
	if (!assin[i])
	    initPt(pts[i]);

    cvReleaseMat(&measurement);
	delete assin;
}


// get a vector of tracked points having above a certain status
vector<VPTrackPt> VPTracker::GetTracks(int minstatus)
{
    vector<VPTrackPt> trackpts;

	for (int i=0; i<(int)tracks.size(); i++)
	{
		if (tracks[i].status >= minstatus)
		{
			VPTrackPt pt;
			pt.x = tracks[i].kalman->state_post->data.fl[0];
			pt.y = tracks[i].kalman->state_post->data.fl[1];
			pt.vx = tracks[i].kalman->state_post->data.fl[2];
			pt.vy = tracks[i].kalman->state_post->data.fl[3];
			pt.status = tracks[i].status;
			pt.nassoc = tracks[i].nassoc;

			trackpts.push_back(pt);
		}
	}

    sort(trackpts.begin(), trackpts.end(), trackpts_comp);

    return trackpts;
}


// create a new point & kf struct
int VPTracker::initPt(CvPoint2D32f pt)
{
    vpTrack newtrack;
    CvKalman* kf = cvCreateKalman(4, 2, 0);

    newtrack.kalman = kf;
    newtrack.status = INIT_STATUS;
    newtrack.dt = 0;
    newtrack.nassoc = 1;

    // set kalman filter properties
    // measurement matrix maps state to measurement, is identity for xy
    cvSetIdentity( kf->measurement_matrix, cvRealScalar(1) );
    
    // want relatively high measurement noise
    cvSetIdentity( kf->measurement_noise_cov, cvRealScalar(KF_MEAS_NOISE) );
    // and a much lower process noise
    cvSetIdentity( kf->process_noise_cov, cvRealScalar(KF_PROC_NOISE) );
    
    // start point out with high uncertainty
    cvSetIdentity( kf->error_cov_post, cvRealScalar(1) );    

    // set initial state, velocity starts at 0
    cvZero( kf->state_post );
    kf->state_post->data.fl[0] = pt.x;
    kf->state_post->data.fl[1] = pt.y;

    // set diagonal of state transition matrix
    cvSetIdentity( kf->transition_matrix, cvRealScalar(1) );

    // now add it
    tracks.push_back(newtrack);

    return (int)tracks.size();
}

// remove point and assoc data
void VPTracker::removePt(int index)
{
    if (tracks.size() == 0)
		return;

    // release matrix
    cvReleaseKalman(&((tracks.begin()+index)->kalman));
    // and remove
    tracks.erase(tracks.begin() + index);
}


VPTracker::VPTracker()
{}

VPTracker::~VPTracker()
{
    while (tracks.size() > 0)
		removePt(0);
}
