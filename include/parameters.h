#pragma once
#ifndef PARAMETERS_H
#define PARAMETERS_H

#endif // PARAMETERS_H

#include<iostream>
#include <vector>
#include<fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cameramodel.h"
using namespace cv;
using namespace std;
extern int NUMOFIMAGES;
extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::vector<std::string> CAM_NAMES;
extern std::string FISHEYE_MASK;
extern int MAX_CNT;
extern double MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern int NUMOFFRAMESTOINIT;
extern int STARTFRAME;
extern bool SHOWIMG;
extern bool IFSHOWPOINTS;
void readParameters(string configdir);
