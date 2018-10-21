#pragma once
#ifndef VIOODOMETRY_H
#define VIOODOMETRY_H

#endif // VIOODOMETRY_H

#include "feature_track.h"
#include "datareading.h"
#include "frame.h"
#include "featurepoints.h"
#include "parameters.h"
#include "solvepose.h"
#include "opencv2/core/core.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/core/eigen.hpp>



using namespace cv;
using namespace std;
using namespace Eigen;



struct ReprojectError{
    ReprojectError(double u,double v)
        :observed_u(u),observed_v(v)
    {}

    template <typename T>
    bool operator()(const T* camera_R,const T* camera_t,const T* point,T* residuals)const{
        T p[3];
        ceres::QuaternionRotatePoint(camera_R,point,p);
        p[0]=p[0]+camera_t[0];
        p[1]=p[1]+camera_t[1];
        p[2]=p[2]+camera_t[2];
        T xp=p[0]/p[2];
        T yp=p[1]/p[2];
        residuals[0]=xp-T(observed_u);
        residuals[1]=yp-T(observed_v);
        return true;
    }

    static ceres::CostFunction* Create(const double observed_x,const double observed_y){
        return(new ceres::AutoDiffCostFunction<ReprojectError,2,4,3,3>(
                   new ReprojectError(observed_x,observed_y)));
    }

    double observed_u;
    double observed_v;
};

class VioOdometry{
public:
    VioOdometry();
    void Track(const Mat& img,double timestamp,int frameid);
    void solvePoseofframesatbegining();
    void trangulatePointsatbegining();
    void optimizatePoseandPointsatbegining();
    void resetPoseofeachframeatbegining();
    void trackFeatures();
    void VioInitilizatin();
    void Sfm();
    void IMUintergration();
    void SlideInitialization();
    void Optimization();
    void ifStartSfm();

    void getMatchedFeatures(int frontframe,int backframe,vector<pair<Point2f,Point2f>> &matchedfeatures);

    void getParallax(double &parallax,vector<pair<Point2f,Point2f>> matchedfeatures);
    bool judgeInitcondition(int curframeid,Mat &relative_R,Mat &relative_t,int &l);
    bool recoverPoseofSFM(int l,int frameid);
    bool solveFrameByPnp(int l);
    void triangulateTwoFrames(int n,int m);
    void triangulatePointByMinsSquare(Matrix<double,4,4> T1,Matrix<double,4,4> T2,vector<Vector3d> pts1,vector<Vector3d> pts2
                                      ,vector<int> indexs);
    void triangulateAllpoints(int startframe,int endframe);
    bool initBaOptimization(int statrframe,int endframe,int l);
    void writeFramepose(string filepath);

    bool optimizatetwoFrames(int first, int second);


    bool StartInitial_flag,Initialized_flag;
    Feature_Track feature_Track;
    vector<Frame> frames;
    vector<FeaturePoints> featurePoints;
    SolvePose solvePose;

};
