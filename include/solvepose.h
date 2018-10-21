#pragma once
#include<Eigen/Dense>
#include <Eigen/Geometry>
#include<Eigen/Core>
#include<opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>
#include"cameramodel.h"
#include <opencv2/core/core.hpp>
#include<vector>
using namespace cv;
using namespace std;
using namespace Eigen;
class SolvePose{
public:
    SolvePose();
    bool solvePoseby2d2d(vector<Point2f> pts1,vector<Point2f> pts2,Mat &R,Mat &t);
    bool solvePoseby2d2d(vector<pair<Point2f, Point2f> > matchedfeatures,Mat &R,Mat &t);
    void solvePosebypnp();
    void triangulatePoint(Matrix<double,3,4> &pose0,Matrix<double,3,4> &pose1,Vector2d &point0,
                          Vector2d &point1,Vector3d &point_3d);



};
