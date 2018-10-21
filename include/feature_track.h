#pragma once
#ifndef FEATURE_TRACK_H
#define FEATURE_TRACK_H

#endif // FEATURE_TRACK_H
#include "frame.h"
#include "featurepoints.h"
#include "parameters.h"
#include "cameramodel.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<stdlib.h>
using namespace cv;
using namespace std;

class Feature_Track
{
public:
    Feature_Track();
    //track feature points
    void trackFeaturePoints(const Mat& img,vector<Frame> &frames,Frame &frame,vector<FeaturePoints> &featurePoints);
    void trackFeaturePoints2(const Mat& img,int frameid,vector<int> featuresidinprecframe,vector<Point2f> &newaddedpoints,
                            vector<pair<int,pair<Point2f,Point2f>>>& trackedpointsincomingframes);
    vector<Frame> getFrames() const;
    void setFrames(const vector<Frame> &value);

    vector<string> getStrimgs() const;
    void setStrimgs(const vector<string> &value);

    vector<string> getStrtimestamps() const;
    void setStrtimestamps(const vector<string> &value);

    Feature_Track(const vector<string> strimg, const vector<double> timestamps);
    void rejectWithF(vector<int> &indexs,vector<Point2f> &un_curp,vector<Point2f> &un_forwp);
    void setMask();
    void goodFeaturesToTrackOwm( InputArray _image, OutputArray _corners,
                                  int maxCorners, double qualityLevel, double minDistance,
                                  InputArray _mask, int blockSize,
                                  bool useHarrisDetector, double harrisK );
    int numofimgs;
    vector<string> strimgs;
    vector<string> strtimestamps;
    vector<cv::Point2f> prevpoint2fs,curpoint2fs;
    vector<cv::Point2f> point2fs;
    Mat previmg,curimg;
    Mat mask;

};
