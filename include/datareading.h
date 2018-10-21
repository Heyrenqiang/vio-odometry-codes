#pragma once
#ifndef DATAREADING_H
#define DATAREADING_H

#endif // DATAREADING_H
#include "parameters.h"
#include <istream>
#include <fstream>
#include <algorithm>
#include "opencv2/core/core.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/core/eigen.hpp>
#include<pangolin/pangolin.h>
#include<sophus/se3.hpp>

#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace Eigen;

typedef vector<Sophus::SE3<double,0>,aligned_allocator<Sophus::SE3<double,0>>> VecSE3;
typedef vector<Vector3d,aligned_allocator<Vector3d>> VecVec3d;

class DataReading{
public:
    DataReading(const string imgdir,const string timestampdir);
    void readImage(vector<string> &strimage, vector<double> &timestamps);
    void readCameraParameters();
    static void writeFramepose();
    static void writePointscoordinates();
    static void writeIMUpose();


    static void showTrajectory(vector<string> filepaths,int startframe,int endframe);

    static void Draw(const vector<VecSE3> &poses_s);




    static void showTrajectoryandpoints(vector<Quaterniond> poses_R,vector<Vector3d> poses_t,vector<Vector3d> points);
private:
    string imgdir;
    string timestampdir;
    string configdir;

};
