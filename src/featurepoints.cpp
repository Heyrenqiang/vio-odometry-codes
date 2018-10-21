#include "featurepoints.h"

int FeaturePoints::id=0;
FeaturePoints::FeaturePoints(){

}

void FeaturePoints::featurManage()
{

}
FeaturePoints::FeaturePoints(Point2f p,int firstsawframe){
    pointid=id;
    id++;

    inWithcframes.push_back(firstsawframe);
    raw_pixelpoints.push_back(make_pair(firstsawframe,p));
    Point3f pnorm,un_pnorm;
    Point2f un_p;
    Cameramodel::pixelToNorm(p,pnorm);
    Cameramodel::undistoredPoints(pnorm,un_pnorm);
    normpoints.push_back(make_pair(firstsawframe,un_pnorm));
    Cameramodel::normToPixel(un_pnorm,un_p);
    pixelpoints.push_back(make_pair(firstsawframe ,un_p));
    Point2f temp;
    temp.x=un_pnorm.x;
    temp.y=un_pnorm.y;
    normpoints2d.push_back(make_pair(firstsawframe,temp));
    triangulated=false;
}

void FeaturePoints::addFrameThatsaw(int idoftheframe,Point2f &p,Point2f &raw_p)
{
    inWithcframes.push_back(idoftheframe);
    raw_pixelpoints.push_back(make_pair(idoftheframe,raw_p));
    pixelpoints.push_back(make_pair(idoftheframe,p));
    Point3f pnorm;
    Cameramodel::pixelToNorm(p,pnorm);
    normpoints.push_back(make_pair(idoftheframe,pnorm));
    Point2f temp;
    temp.x=pnorm.x;
    temp.y=pnorm.y;
    normpoints2d.push_back(make_pair(idoftheframe,temp));
}

