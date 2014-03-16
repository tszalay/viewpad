/***************************
 *** POINT TRACKER CLASS ***
 ***************************/

// Created by Tamas Szalay
// May 23, 2010

#include "stdafx.h"

#ifndef __VPTRACKER_H__
#define __VPTRACKER_H__

using namespace std;


// external tracked point struct we return
struct VPTrackPt
{
    float x;
    float y;
    float vx;
    float vy;
    int status;
    int nassoc;
};


// internal struct used for data
struct vpTrack
{
    CvKalman* kalman;
    int status;
    int nassoc;
    float dt;
};

class VPTracker
{
public:
    VPTracker();
    ~VPTracker();
    
    void Update(vector<CvPoint2D32f> pts, float dt);

    vector<VPTrackPt> GetTracks(int minstatus);
    

private:

    // vector containing all of our tracks
    vector<vpTrack> tracks;

  
    // initialize a new point (status and kalman struct)
    // returns point index
    int initPt(CvPoint2D32f pt);

    // remove a point
    void removePt(int index);
};

#endif
