#pragma once
#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
#include <Eigen/Geometry>
#include<opencv2/core/core.hpp>
using namespace Eigen;
using namespace cv;
class Tools{
public:
    Tools();
    static Matrix3d vector2matrix(Mat &t);
};
Tools::Tools(){

}
Matrix3d Tools::vector2matrix(Mat &t){
    double a=t.at<double>(0,0),b=t.at<double>(1,0),c=t.at<double>(2,0);
    Matrix3d m3d;
//    m3d<< 0, -c, b,
//          c, 0, -a,
//          -b, a, 0;
    return m3d;
}
