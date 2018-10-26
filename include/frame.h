#ifndef FRAME_H
#define FRAME_H

#endif // FRAME_H
#pragma once
#include"featurepoints.h"
#include<map>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<Eigen/Core>
#include<opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;
using namespace Eigen;
//camera frame
class Frame
{
public:
    Frame(const double timestamp,const int id);

    Frame();


    void addFeaturePoints(vector<FeaturePoints> featurepoints);
    void addFeaturePoints(FeaturePoints featurepoint);
    void addTrackedFeaturePointstonext(int id,Point2f p);
    void addTrackedFeaturePointstoprevious(int id,Point2f p);
    void setpose(Matrix3d &R,Vector3d &t);
    void addPreInformation(vector<Vector3d> slic_accs,vector<Vector3d> slic_omigas,vector<double> slic_imustamps);

    bool posesolved;
    int id;
    Mat Mat33_pose_R,Mat31_pose_t;

    Mat Mat34_pose_Rt;
    Mat Mat44_pose_T;

    Matrix3d M3d_pose_R;
    Vector3d V3d_pose_t;

    Quaterniond q_pose_R;
    Quaterniond inv_q_pose_R;

    Matrix<double,3,4> M34_pose_Rt;
    Matrix4d M4d_pose_T;

    Matrix4d inv_M4d_pose_T;
    Matrix<double,3,4> inv_M34_pose_Rt;
    Matrix3d inv_M3d_pose_R;
    Vector3d inv_V3d_pose_t;

    Mat inv_Mat44_pose_T;
    Mat inv_Mat34_pose_Rt;
    Mat inv_Mat33_pose_R;
    Mat inv_Mat31_pose_t;

    double q_pose_forba[4];
    double t_pose_forba[3];

    double timestamp;
    vector<int> features;
    vector<pair<int,Point2f>> featurePointscoords;
    vector<pair<int,Point3f>> featurePointscoords_norm;
    vector<pair<int,Point2f>> featurePointscoords_norm_2d;
    vector<pair<int,Point2f>> trackedFeaturePointstonext;
    vector<pair<int,Point2f>> trackedFeaturePointstoprevious;
    vector<Point2f> trackedPointstonext;
    vector<Point2f> trackedPointstoprev;

    //gyr and acc bias
    Vector3d acc_bias;
    Vector3d gyr_bias;
    //delta gyr and acc bias,achived by alian
    Vector3d delta_gyr_bias;
    Vector3d delta_acc_bias;

    //preintsgration item
    Quaterniond pre_integration_pose_gama;
    Vector3d pre_integration_pose_alpha;
    Vector3d pre_integration_pose_beta;

    MatrixXd jacobian;
    MatrixXd covariance;

    //imu data between two this frame and last frame
    vector<Vector3d> slic_accs;
    vector<Vector3d> slic_omigas;;
    vector<double> slic_imustamps;

    bool hasenoughimudata;
};
