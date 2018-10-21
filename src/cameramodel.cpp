#include"cameramodel.h"
int Cameramodel::imagewidth=0;
int Cameramodel::imageheight=0;
float Cameramodel::k1=0;
float Cameramodel::k2=0;
float Cameramodel::p1=0;
float Cameramodel::p2=0;
float Cameramodel::fx=0;
float Cameramodel::fy=0;
float Cameramodel::cx=0;
float Cameramodel::cy=0;
Cameramodel::Cameramodel(){

}

void Cameramodel::pixelToNorm(Point2f p, Point3f &pnorm)
{

    pnorm.x=(p.x-cx)/fx;
    pnorm.y=(p.y-cy)/fy;
    pnorm.z=1.0;
}

void Cameramodel::normToPixel(Point3f pnorm, Point2f &p)
{
    p.x=fx*pnorm.x+cx;
    p.y=fy*pnorm.y+cy;
}

void Cameramodel::undistoredPoints(Point3f pnorm, Point3f &un_pnorm)
{
    float x,y,x2,y2,r2,r4,xy;
    x=pnorm.x;
    y=pnorm.y;
    x2=x*x;
    y2=y*y;
    xy=x*y;
    r2=x2+y2;
    r4=r2*r2;
    un_pnorm.x=x*(1+k1*r2+k2*r4)+2*p1*x*y+p2*(r2+2*x2);
    un_pnorm.y=y*(1+k1*r2+k2*r4)+p1*(r2+2*y2)+2*p2*x*y;
    un_pnorm.z=1.0;
}

void Cameramodel::removeDistortion(Point2f p, Point2f &un_p)
{
    Point3f pnorm,un_pnorm;
    pixelToNorm(p,pnorm);
    undistoredPoints(pnorm,un_pnorm);
    normToPixel(un_pnorm,un_p);
}
