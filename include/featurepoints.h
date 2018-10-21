#pragma once
#ifndef FEATUREPOINTS_H
#define FEATUREPOINTS_H

#endif // FEATUREPOINTS_H
#include "cameramodel.h"
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
using namespace std;
using namespace cv;
using namespace Eigen;


class FeaturePoints
{
public:
    FeaturePoints();

    static int id;
    int pointid;

    vector<pair<int,Point2f>> raw_pixelpoints;
    vector<pair<int,Point2f>> pixelpoints;
    vector<pair<int,Point3f>> normpoints;
    vector<pair<int,Point2f>> normpoints2d;
    vector<int> inWithcframes;

    bool triangulated;

    Vector3d point3d_Eigen;
    Point3f point3d_Point3f;

    double point3d_forba[3];

    void featurManage();
    FeaturePoints(Point2f p,int firstsawframe);
    void addFrameThatsaw(int idoftheframe,Point2f &p,Point2f &raw_p);
};
