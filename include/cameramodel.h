#pragma once
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
using namespace cv;
class Cameramodel{
public:
    Cameramodel();
    static void pixelToNorm(Point2f p,Point3f &pnorm);
    static void normToPixel(Point3f pnorm,Point2f &p);
    static void pixelToSpace();
    static void spaceToPixel();
    static void undistoredPoints(Point3f pnorm,Point3f &un_pnorm);
    static void removeDistortion(Point2f p,Point2f &un_p);

    static int imagewidth;
    static int imageheight;
    static float k1,k2,p1,p2;
    static float fx,fy,cx,cy;
};
