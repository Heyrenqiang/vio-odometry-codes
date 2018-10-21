#include "frame.h"
Frame::Frame(){

}


void Frame::addFeaturePoints(FeaturePoints featurepoint)
{
    //featurepointsinframe.push_back(make_pair(featurepoint.pointid,featurepoint));
    featurePointscoords.push_back(make_pair(featurepoint.pointid,featurepoint.pixelpoints.back().second));
    featurePointscoords_norm.push_back(make_pair(featurepoint.pointid,featurepoint.normpoints.back().second));
    featurePointscoords_norm_2d.push_back(make_pair(featurepoint.pointid,featurepoint.normpoints2d.back().second));
}

void Frame::addTrackedFeaturePointstoprevious(int id,Point2f p)
{
    trackedFeaturePointstoprevious.push_back(make_pair(id,p));
    trackedPointstoprev.push_back(p);
}

void Frame::setpose(Matrix3d &R,Vector3d &t)
{
    M3d_pose_R=R;
    V3d_pose_t=t;
    q_pose_R=Quaterniond(M3d_pose_R);
    inv_q_pose_R=q_pose_R.inverse();
    M34_pose_Rt.block<3,3>(0,0)=M3d_pose_R;
    M34_pose_Rt.block<3,1>(0,3)=V3d_pose_t;
    M4d_pose_T.block<3,4>(0,0)=M34_pose_Rt;
    M4d_pose_T.block<1,4>(3,0)=Matrix<double,1,4>(0,0,0,1);
    eigen2cv(M3d_pose_R,Mat33_pose_R);
    eigen2cv(V3d_pose_t,Mat31_pose_t);
    eigen2cv(M4d_pose_T,Mat44_pose_T);
    Mat34_pose_Rt=(Mat_<double>(3,4)<<
                Mat33_pose_R.at<double>(0,0),Mat33_pose_R.at<double>(0,1),Mat33_pose_R.at<double>(0,2),Mat31_pose_t.at<double>(0,0),
                Mat33_pose_R.at<double>(1,0),Mat33_pose_R.at<double>(1,1),Mat33_pose_R.at<double>(1,2),Mat31_pose_t.at<double>(1,0),
                Mat33_pose_R.at<double>(2,0),Mat33_pose_R.at<double>(2,1),Mat33_pose_R.at<double>(2,2),Mat31_pose_t.at<double>(2,0));
    Mat44_pose_T=(Mat_<double>(4,4)<<
                Mat33_pose_R.at<double>(0,0),Mat33_pose_R.at<double>(0,1),Mat33_pose_R.at<double>(0,2),Mat31_pose_t.at<double>(0,0),
                Mat33_pose_R.at<double>(1,0),Mat33_pose_R.at<double>(1,1),Mat33_pose_R.at<double>(1,2),Mat31_pose_t.at<double>(1,0),
                Mat33_pose_R.at<double>(2,0),Mat33_pose_R.at<double>(2,1),Mat33_pose_R.at<double>(2,2),Mat31_pose_t.at<double>(2,0),
                0,0,0,1);
    inv_M4d_pose_T=M4d_pose_T.inverse();
    inv_M34_pose_Rt=inv_M4d_pose_T.block<3,4>(0,0);
    inv_M3d_pose_R=inv_M4d_pose_T.block<3,3>(0,0);
    inv_V3d_pose_t=inv_M4d_pose_T.block<3,1>(0,3);

    eigen2cv(inv_M4d_pose_T,inv_Mat44_pose_T);
    eigen2cv(inv_M34_pose_Rt,inv_Mat34_pose_Rt);
    eigen2cv(inv_M3d_pose_R,inv_Mat33_pose_R);
    eigen2cv(inv_V3d_pose_t,inv_Mat31_pose_t);

}

void Frame::addTrackedFeaturePointstonext(int id,Point2f p)
{
    trackedFeaturePointstonext.push_back(make_pair(id,p));
    trackedPointstonext.push_back(p);
}

Frame::Frame( const double timestamp, const int id):
   id(id), timestamp(timestamp)
{
    posesolved=false;
}
