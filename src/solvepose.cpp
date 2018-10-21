#include "solvepose.h"

SolvePose::SolvePose()
{

}

bool SolvePose::solvePoseby2d2d(vector<Point2f> pts1, vector<Point2f> pts2, Mat &R, Mat &t)
{
    if(pts1.size()>=15){

        Mat mask;
        Mat cameraMatrix=(Mat_<double>(3,3)<<Cameramodel::fx,0,Cameramodel::cx,
                          0,Cameramodel::fy,Cameramodel::cy,
                          0,0,1);

        Mat E=findEssentialMat(pts1,pts2,cameraMatrix,RANSAC,0.999,1.0,mask);

        int inlier_cnt=recoverPose(E,pts1,pts2,cameraMatrix,R,t,mask);
        if(inlier_cnt>12){
            return true;
        }else{
            return false;
        }
    }
    return false;
}

bool SolvePose::solvePoseby2d2d(vector<pair<Point2f, Point2f> > matchedfeatures, Mat &R, Mat &t)
{
    if(matchedfeatures.size()>=15){
        vector<Point2f> pts1,pts2;
        for(size_t i=0;i<matchedfeatures.size();i++){
            pts1.push_back(matchedfeatures[i].first);
            pts2.push_back(matchedfeatures[i].second);
        }
        Mat mask;
        Mat cameraMatrix=(Mat_<double>(3,3)<<Cameramodel::fx,0,Cameramodel::cx,
                          0,Cameramodel::fy,Cameramodel::cy,
                          0,0,1);

        Mat E=findEssentialMat(pts1,pts2,cameraMatrix,RANSAC,0.999,1.0,mask);

        int inlier_cnt=recoverPose(E,pts1,pts2,cameraMatrix,R,t,mask);

        if(inlier_cnt>12){
//            Matrix3d mr;
//            Vector3d vt;
//            cv2eigen(R,mr);
//            cv2eigen(t,vt);
//            Matrix3d vthat;
//            double a=t.at<double>(0,0),b=t.at<double>(1,0),c=t.at<double>(2,0);
//            vthat<<
//                   0,-c,b,
//                   c,0,-a,
//                   -b,a,0;
//            Matrix3d K;
//            cv2eigen(cameraMatrix,K);
//            cout<<"R:"<<mr<<endl;
//            cout<<"t:"<<vt<<endl;
            for(size_t k=0;k<pts1.size();k++){

//                Vector3d pv1(pts1[k].x,pts1[k].y,1);
//                Vector3d pv2(pts2[k].x,pts2[k].y,1);

//                Vector3d x1=K.inverse()*pv1;
//                Vector3d x2=K.inverse()*pv2;

//                Matrix<double,1,1> err=pv2.transpose()*K.inverse().transpose()*vthat*mr*K.inverse()*pv1;
//                cout<<"err:"<<err<<endl;
//                //cout<<"pv1:"<<pv1<<endl;
//                //cout<<"pv2:"<<pv2<<endl;
//                cout<<"x1:"<<x1<<endl;
//                cout<<"x2:"<<x2<<endl;
//                cout<<"x1_pie:"<<mr*x2+vt<<endl;
//                cout<<"x2_pie:"<<mr*x1+vt<<endl;
//                cout<<"...."<<endl;


            }
            cout<<R<<t<<endl;

            return true;
        }else{
            return false;
        }
    }
    return false;
}

void SolvePose::solvePosebypnp()
{

}

void SolvePose::triangulatePoint(Matrix<double, 3, 4> &pose0, Matrix<double, 3, 4> &pose1,
                                 Vector2d &point0, Vector2d &point1, Vector3d &point_3d)
{
    Matrix4d design_matrix=Matrix4d::Zero();
    design_matrix.row(0)=point0[0]*pose0.row(2)-pose0.row(0);
    design_matrix.row(1)=point1[1]*pose0.row(2)-pose0.row(1);
    design_matrix.row(2)=point1[0]*pose1.row(2)-pose1.row(0);
    design_matrix.row(3)=point1[1]*pose1.row(2)-pose1.row(1);
    Vector4d triangulate_point;
    triangulate_point=design_matrix.jacobiSvd(ComputeFullV).matrixV().rightCols<1>();
    point_3d(0)=triangulate_point(0)/triangulate_point(3);
    point_3d(1)=triangulate_point(1)/triangulate_point(3);
    point_3d(2)=triangulate_point(2)/triangulate_point(3);
}

