#include "vioodometry.h"

#include <vioodometry.h>



VioOdometry::VioOdometry()
{

}

void VioOdometry::Track(const Mat& img, double timestamp, int frameid)
{
    //creat frame by image and label it by frameid
    Frame frame(timestamp,frameid);
    frames.push_back(frame);

    //track the feature frame by frame and creat the featurepoints,
    //each frame saw severalfeature points,each points saw sevaral frames
    //feature_Track.trackFeaturePoints(img,frames,frames[frames.size()-1],featurePoints);
    vector<int> pointsinprevframes;
    if(frameid==0){
        pointsinprevframes.clear();
    }else{
       pointsinprevframes.clear();
        for(size_t i=0;i<frames[frames.size()-2].featurePointscoords_norm.size();i++){
            pointsinprevframes.push_back(
                        frames[frames.size()-2].featurePointscoords_norm[i].first);
        }
    }
    vector<Point2f> newaddedpoints;
    vector<pair<int,pair<Point2f,Point2f>>> trackedpointsincomingframes;
    feature_Track.trackFeaturePoints2(img,frameid,pointsinprevframes,newaddedpoints,trackedpointsincomingframes);
    for(size_t i=0;i<trackedpointsincomingframes.size();i++){
        ulong index=ulong(trackedpointsincomingframes[i].first);
        Point2f p=trackedpointsincomingframes[i].second.first;
        Point2f raw_p=trackedpointsincomingframes[i].second.second;
        featurePoints[index].addFrameThatsaw(frameid,p,raw_p);
        frames[ulong(frameid)].addFeaturePoints(featurePoints[index]);
    }
    for(size_t i=0;i<newaddedpoints.size();i++){
        FeaturePoints featurepoint(newaddedpoints[i],frameid);
        featurePoints.push_back(featurepoint);
        frames[ulong(frameid)].addFeaturePoints(featurepoint);
    }
    if(SHOWIMG){

        Mat imgshow=img.clone();
        for(auto kp: frame.featurePointscoords){
            circle(imgshow,kp.second,4,Scalar(0,240,0),3);
        }
        imshow("corners",imgshow);
        waitKey(0);
    }
    /*
     * add code to judge if imu ready or frame numbers enough;
    */
    if(frameid>STARTFRAME){
        StartInitial_flag=true;
    }else{
        StartInitial_flag=false;
    }
    if(StartInitial_flag){
        Mat relative_R,relative_t;
        int l;
        //judge if last frame has goog correspondence with front frames;
        if(judgeInitcondition(frameid,relative_R,relative_t,l)){

            //initial frame pose and featurepoints;
            recoverPoseofSFM(l,frameid);
            writeFramepose("../sfm-project/data/trajectory.txt");
            //initial ba optimization.
            if(initBaOptimization(frameid-NUMOFFRAMESTOINIT+1,frameid,l)){
                writeFramepose("../sfm-project/data/trajectoryafterba.txt");
                cout<<"init ba successed."<<endl;
            }


            vector<string> paths;
            paths.push_back("../sfm-project/data/trajectory.txt");
            paths.push_back("../sfm-project/data/trajectoryafterba.txt");
            paths.push_back("../sfm-project/data/groundtruth.txt");
            DataReading::showTrajectory(paths,0,10000);


            vector<Quaterniond> poses_R;
            vector<Vector3d> poses_t;
            vector<Vector3d> points;
            for(size_t i=0;i<=frames.size();i++){
                if(!frames[i].posesolved)
                    continue;
                poses_R.push_back(frames[i].q_pose_R);
                poses_t.push_back(frames[i].V3d_pose_t);
            }
            for(size_t i=0;i<featurePoints.size();i++){
                if(!featurePoints[i].triangulated)
                    continue;
                points.push_back(featurePoints[i].point3d_Eigen);
            }
            DataReading::showTrajectoryandpoints(poses_R,poses_t,points);
            cout<<"end."<<endl;
        }else{
           /*do nothing
            *
            */
        }
    }
}

void VioOdometry::solvePoseofframesatbegining()
{
    size_t l=frames.size()-size_t(NUMOFFRAMESTOINIT);
    size_t lend=frames.size();
    for(size_t i=l;i<lend-1;i++){
        Mat R,t;
        solvePose.solvePoseby2d2d(frames[i].trackedPointstonext,frames[i+1].trackedPointstoprev,R,t);

    }
}

void VioOdometry::trackFeatures()
{

}
//get matched features between two frames
void VioOdometry::getMatchedFeatures(int frontframe, int backframe, vector<pair<Point2f, Point2f> > &matchedfeatures)
{
    int p_star=frames[ulong(frontframe)].featurePointscoords[0].first;
    int p_end=frames[ulong(backframe)].featurePointscoords.back().first;
    for(int i=p_star;i<=p_end;i++){
        if(featurePoints[ulong(i)].inWithcframes.front()<=int(frontframe)&&featurePoints[ulong(i)].inWithcframes.back()>=int(backframe)){
            Point2f p1;
            for(size_t k=0;k<featurePoints[ulong(i)].pixelpoints.size();k++){
                if(featurePoints[ulong(i)].pixelpoints[k].first==frontframe){
                    p1=featurePoints[ulong(i)].pixelpoints[k].second;
                    break;
                }
            }
            //cout<<i<<endl;
            Point2f p2=featurePoints[ulong(i)].pixelpoints.back().second;
            matchedfeatures.push_back(make_pair(p1,p2));
        }
    }
}
//get parallax between two group of matched features
void VioOdometry::getParallax(double &parallax, vector<pair<Point2f, Point2f> > matchedfeatures)
{
    double sumparallax=0;
    for(size_t i=0;i<matchedfeatures.size();i++){
        Vector2d p1(matchedfeatures[i].first.x,matchedfeatures[i].first.y);
        Vector2d p2(matchedfeatures[i].second.x,matchedfeatures[i].second.y);
        double temp=(p1-p2).norm();
        sumparallax=sumparallax+temp;
    }
    parallax=sumparallax/matchedfeatures.size()*460;
}
//judge if the last frame have good correspondence with front frames
bool VioOdometry::judgeInitcondition(int curframeid,Mat &relative_R,Mat &relative_t,int &l)
{
    for(int i=curframeid-NUMOFFRAMESTOINIT+1;i<curframeid;i++){

        vector<pair<Point2f, Point2f> > matchedfeatures;
        getMatchedFeatures(i,curframeid,matchedfeatures);
        double parallax;
        getParallax(parallax,matchedfeatures);
        if(parallax>30&&solvePose.solvePoseby2d2d(matchedfeatures,relative_R,relative_t)){
            l=i;
            frames[ulong(l)].posesolved=true;
            frames[ulong(curframeid)].posesolved=true;
            Mat temp_R=(Mat_<double>(3,3)<<1,0,0,0,1,0,0,0,1);
            Mat temp_t=(Mat_<double>(3,1)<<0,0,0);
            Mat relative_R_t=relative_R.inv();
            Mat relative_t_t=-relative_R_t*relative_t;
            Matrix3d Rl,Rcur;
            Vector3d tl,tcur;
            for(int k=0;k<3;k++){
                tl(k)=temp_t.at<double>(k);
                tcur(k)=relative_t_t.at<double>(k);
                for(int j=0;j<3;j++){
                    Rl(k,j)=temp_R.at<double>(k,j);
                    Rcur(k,j)=relative_R_t.at<double>(k,j);
                }
            }
            frames[ulong(i)].setpose(Rl,tl);
            frames[ulong(curframeid)].setpose(Rcur,tcur);
            //triangulateTwoFrames(l,curframeid);
            return true;
        }
    }
    return false;
}

bool VioOdometry::recoverPoseofSFM(int l, int frameid)
{
    for(int i=l;i<frameid;i++){
        if(i>l){
            if(!solveFrameByPnp(i)){
                return false;
            }
        }
        triangulateTwoFrames(i,frameid);
        if(i==l){
            if(!optimizatetwoFrames(i,frameid)){
                return false;
            }
        }
    }
    //go on recover feature space points;
    for(int i=l+1;i<frameid;i++){
        triangulateTwoFrames(l,i);
    }

    for(int i=l-1;i>frameid-NUMOFFRAMESTOINIT;i--){
        if(!solveFrameByPnp(i)){
            return false;
        }
        triangulateTwoFrames(i,l);
    }
    for(int i=frameid-NUMOFFRAMESTOINIT+1;i<frameid;i++){
        triangulateTwoFrames(i,i+1);
    }
    //triangulateAllpoints(frameid-NUMOFFRAMESTOINIT+1,frameid);
    cout<<"SFM init succed!"<<endl;
    return true;
}

bool VioOdometry::solveFrameByPnp(int l)
{
    if(l==889){
        cout<<"893"<<endl;
    }
    //cout<<"hell world"<<endl;
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    vector<Point3f> pts_3d_norm;
    //find corresponding feature space points and pixel points;
    for(size_t i=0;i<frames[ulong(l)].featurePointscoords.size();i++){
        int index=frames[ulong(l)].featurePointscoords[i].first;
        if(featurePoints[ulong(index)].triangulated){
            pts_2d.push_back(frames[ulong(l)].featurePointscoords_norm_2d[i].second);
            pts_3d_norm.push_back(frames[ulong(l)].featurePointscoords_norm[i].second);
            pts_3d.push_back(featurePoints[ulong(index)].point3d_Point3f);
        }
    }
    if(pts_2d.size()<15){
        cout<<"unstable feature tracking"<<endl;
        if(pts_2d.size()<10){
            return false;
        }
    }
    Mat r,revc,t,D,tmp_r;

    //set initial r t.
    if(frames[ulong(l-1)].posesolved){
        tmp_r=frames[ulong(l-1)].Mat33_pose_R;
        Rodrigues(tmp_r,revc);
        t=frames[ulong(l-1)].Mat31_pose_t;
    }else if(frames[ulong(l+1)].posesolved){
        tmp_r=frames[ulong(l+1)].Mat33_pose_R;
        Rodrigues(tmp_r,revc);
        t=frames[ulong(l+1)].Mat31_pose_t;
    }

    Mat K=(Mat_<double>(3,3)<<1,0,0,0,1,0,0,0,1);
    bool pnpsucc;
    //solve pnp
    pnpsucc=solvePnP(pts_3d,pts_2d,K,D,revc,t,1);
    if(!pnpsucc){
        return false;
    }
    Rodrigues(revc,r);
    Matrix3d pose_R;
    Vector3d pose_t;
    cv2eigen(r,pose_R);
    cv2eigen(t,pose_t);
    Matrix3d inv_pose_R=pose_R.inverse();
    Vector3d inv_pose_t=-inv_pose_R*pose_t;
    frames[ulong(l)].setpose(inv_pose_R,inv_pose_t);
    frames[ulong(l)].posesolved=true;
    return true;

}
//triangulate the featurepoints between two frames.
void VioOdometry::triangulateTwoFrames(int n, int m)
{
    //judge if the feature appears in the two frame.
    int p_star=frames[ulong(n)].featurePointscoords[0].first;
    int p_end=frames[ulong(m)].featurePointscoords.back().first;
    vector<Vector2d> pn,pm;
    vector<Point2f> pnf,pmf;
    vector<int> indexs;
    //find features in two frames;
    for(int i=p_star;i<=p_end;i++){
        if(featurePoints[ulong(i)].triangulated)
            continue;
        bool has_n=false,has_m=false;
        Point2f tempn,tempm;
        int tempindexn=-1,tempindexm = -1;
        for(size_t j=0;j<featurePoints[ulong(i)].inWithcframes.size();j++){
            if(featurePoints[ulong(i)].inWithcframes[j]==n){
                //pn=Vector2d(featurePoints[ulong(i)].pixelpoints[j].second.x,featurePoints[ulong(i)].pixelpoints[j].second.y);
                tempn=featurePoints[ulong(i)].normpoints2d[j].second;
                tempindexn=int(j);
                has_n=true;
            }
            if(featurePoints[ulong(i)].inWithcframes[j]==m){
                //pn=Vector2d(featurePoints[ulong(i)].pixelpoints[j].second.x,featurePoints[ulong(i)].pixelpoints[j].second.y);
                tempm=featurePoints[ulong(i)].pixelpoints[j].second;
                tempindexm=int(j);
                has_m=true;
            }
        }
        if(has_n&&has_m){
            indexs.push_back(i);
            pnf.push_back(featurePoints[ulong(i)].normpoints2d[ulong(tempindexn)].second);
            pmf.push_back(featurePoints[ulong(i)].normpoints2d[ulong(tempindexm)].second);
        }

    }
    Matrix<double,4,4> T1=frames[ulong(n)].M4d_pose_T;
    Matrix<double,4,4> T2=frames[ulong(m)].M4d_pose_T;
    vector<Vector3d> pts1,pts2;
    for(size_t i=0;i<indexs.size();i++){
        Vector3d temp1(static_cast<double>(pnf[i].x),static_cast<double>(pnf[i].y),1);
        Vector3d temp2(static_cast<double>(pmf[i].x),static_cast<double>(pmf[i].y),1);
        pts1.push_back(temp1);
        pts2.push_back(temp2);
    }
    if(indexs.size()>0){
        Mat p4d;
        triangulatePoints(frames[ulong(n)].inv_Mat34_pose_Rt,frames[ulong(m)].inv_Mat34_pose_Rt,pnf,pmf,p4d);
        //cout<<p4d<<endl;
        for(int i=0;i<p4d.cols;i++){
            Mat x=p4d.col(i);
            //cout<<x<<endl;
            MatrixXd xtemp;
            cv2eigen(x,xtemp);
            xtemp=xtemp/(xtemp(3,0));
            //cout<<xtemp<<endl;
            Point3d p(xtemp(0,0),
                      xtemp(1,0),
                      xtemp(2,0));
            //cout<<p<<endl;
            featurePoints[ulong(indexs[ulong(i)])].point3d_Point3f=p;
            Vector3d peigen(p.x,p.y,p.z);
            featurePoints[ulong(indexs[ulong(i)])].point3d_Eigen=peigen;
            featurePoints[ulong(indexs[ulong(i)])].triangulated=true;

        }
    }
    cout<<"triangulate end."<<endl;

}
void VioOdometry::triangulateAllpoints(int startframe, int endframe)
{

    int startpoint=frames[ulong(startframe)].featurePointscoords_norm.front().first;
    int endpoint=frames[ulong(endframe)].featurePointscoords_norm.back().first;
    int t1=0,t2=0;
    for(int i=startpoint;i<=endpoint;i++){
        if(featurePoints[ulong(i)].triangulated){
            t1++;
            continue;
        }else if(featurePoints[ulong(i)].inWithcframes.size()>=2
                 &&featurePoints[ulong(i)].inWithcframes.back()>startframe){
            t2++;
        }
    }
    cout<<"end."<<endl;
}
bool VioOdometry::optimizatetwoFrames(int first,int second){
    //sfm optimization
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization=new ceres::QuaternionParameterization();
    map<int,double*> Rposeofframes;
    map<int,double*> tposeofframes;
    map<int,double*>  coordinateofpoints;
    double* tempR ;
    double* tempt ;
    //add two frame paramenters
    tempR = frames[ulong(first)].q_pose_forba ;
    tempt = frames[ulong(first)].t_pose_forba ;
    tempR[0]=frames[ulong(first)].inv_q_pose_R.w();
    tempR[1]=frames[ulong(first)].inv_q_pose_R.x();
    tempR[2]=frames[ulong(first)].inv_q_pose_R.y();
    tempR[3]=frames[ulong(first)].inv_q_pose_R.z();
    tempt[0]=frames[ulong(first)].inv_V3d_pose_t.x();
    tempt[1]=frames[ulong(first)].inv_V3d_pose_t.y();
    tempt[2]=frames[ulong(first)].inv_V3d_pose_t.z();
    Rposeofframes.insert(make_pair(first,tempR));
    tposeofframes.insert(make_pair(first,tempt));
    tempR = frames[ulong(second)].q_pose_forba ;
    tempt = frames[ulong(second)].t_pose_forba ;
    tempR[0]=frames[ulong(second)].inv_q_pose_R.w();
    tempR[1]=frames[ulong(second)].inv_q_pose_R.x();
    tempR[2]=frames[ulong(second)].inv_q_pose_R.y();
    tempR[3]=frames[ulong(second)].inv_q_pose_R.z();
    tempt[0]=frames[ulong(second)].inv_V3d_pose_t.x();
    tempt[1]=frames[ulong(second)].inv_V3d_pose_t.y();
    tempt[2]=frames[ulong(second)].inv_V3d_pose_t.z();
    Rposeofframes.insert(make_pair(second,tempR));
    tposeofframes.insert(make_pair(second,tempt));
    problem.AddParameterBlock(Rposeofframes[first],4,local_parameterization);
    problem.AddParameterBlock(tposeofframes[first],3);
    problem.AddParameterBlock(Rposeofframes[second],4,local_parameterization);
    problem.AddParameterBlock(tposeofframes[second],3);
    problem.SetParameterBlockConstant(Rposeofframes[first]);
    problem.SetParameterBlockConstant(tposeofframes[first]);

    int pointstart=frames[ulong(first)].featurePointscoords_norm.front().first;
    int pointend=frames[ulong(second)].featurePointscoords_norm.back().first;
    for(int i=pointstart;i<=pointend;i++){
        if(featurePoints[ulong(i)].triangulated==false)
            continue;
        //add parameter block.
        double* temppoint = featurePoints[ulong(i)].point3d_forba ;
        temppoint[0]=featurePoints[ulong(i)].point3d_Eigen.x();
        temppoint[1]=featurePoints[ulong(i)].point3d_Eigen.y();
        temppoint[2]=featurePoints[ulong(i)].point3d_Eigen.z();
        coordinateofpoints.insert(make_pair(i,temppoint));
        problem.AddParameterBlock(coordinateofpoints[i],3);
        //problem.SetParameterBlockConstant(coordinateofpoints[i]);
        for(size_t j=0;j<featurePoints[ulong(i)].inWithcframes.size();j++){
            int frameindex=featurePoints[ulong(i)].inWithcframes[ulong(j)];
            if(frames[ulong(frameindex)].posesolved==false)
                continue;
            ceres::CostFunction* costfunction=ReprojectError::Create(
                        static_cast<double>(featurePoints[ulong(i)].normpoints[j].second.x),
                        static_cast<double>(featurePoints[ulong(i)].normpoints[j].second.y));
            problem.AddResidualBlock(costfunction,nullptr,
                                     Rposeofframes[frameindex],
                                     tposeofframes[frameindex],
                                     coordinateofpoints[i]);
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds=100;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    if(summary.termination_type==ceres::CONVERGENCE||summary.final_cost<5e-3){
        cout<<summary.BriefReport()<<endl;
        cout<<"ba converge."<<endl;
    }else{
        cout<<"ba failed"<<endl;
        //return false;
    }

    //update pose
    Quaterniond q;
    q.w()=frames[ulong(first)].q_pose_forba[0];
    q.x()=frames[ulong(first)].q_pose_forba[1];
    q.y()=frames[ulong(first)].q_pose_forba[2];
    q.z()=frames[ulong(first)].q_pose_forba[3];
    q=q.inverse();
    Matrix3d r33d(q);
    Vector3d v3d(frames[ulong(first)].t_pose_forba[0],frames[ulong(first)].t_pose_forba[1],
            frames[ulong(first)].t_pose_forba[2]);
    v3d=-r33d*v3d;
    frames[ulong(first)].setpose(r33d,v3d);

    q.w()=frames[ulong(second)].q_pose_forba[0];
    q.x()=frames[ulong(second)].q_pose_forba[1];
    q.y()=frames[ulong(second)].q_pose_forba[2];
    q.z()=frames[ulong(second)].q_pose_forba[3];

    q=q.inverse();
    Matrix3d r33d2(q);
    Vector3d v3d2(frames[ulong(second)].t_pose_forba[0],frames[ulong(second)].t_pose_forba[1],
            frames[ulong(second)].t_pose_forba[2]);
    v3d2=-r33d2*v3d2;
    frames[ulong(second)].setpose(r33d2,v3d2);

    //update points
    for(int i=pointstart;i<=pointend;i++){
        if(featurePoints[ulong(i)].triangulated==false)
            continue;
        featurePoints[ulong(i)].point3d_Eigen.x()=featurePoints[ulong(i)].point3d_forba[0];
        featurePoints[ulong(i)].point3d_Eigen.y()=featurePoints[ulong(i)].point3d_forba[1];
        featurePoints[ulong(i)].point3d_Eigen.z()=featurePoints[ulong(i)].point3d_forba[2];
        featurePoints[ulong(i)].point3d_Point3f.x=static_cast<float>(featurePoints[ulong(i)].point3d_forba[0]);
        featurePoints[ulong(i)].point3d_Point3f.y=static_cast<float>(featurePoints[ulong(i)].point3d_forba[1]);
        featurePoints[ulong(i)].point3d_Point3f.z=static_cast<float>(featurePoints[ulong(i)].point3d_forba[2]);

    }

    return true;
}

bool VioOdometry::initBaOptimization(int statrframe, int endframe,int l)
{

    //two frame optimization

    //sfm optimization
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization=new ceres::QuaternionParameterization();
    map<int,double*> Rposeofframes;
    map<int,double*> tposeofframes;
    map<int,double*>  coordinateofpoints;


    for(int i=statrframe;i<=endframe;i++){
        //add problem block.
        double* tempR = frames[ulong(i)].q_pose_forba ;
        double* tempt = frames[ulong(i)].t_pose_forba ;
        tempR[0]=frames[ulong(i)].inv_q_pose_R.w();
        tempR[1]=frames[ulong(i)].inv_q_pose_R.x();
        tempR[2]=frames[ulong(i)].inv_q_pose_R.y();
        tempR[3]=frames[ulong(i)].inv_q_pose_R.z();
        tempt[0]=frames[ulong(i)].inv_V3d_pose_t.x();
        tempt[1]=frames[ulong(i)].inv_V3d_pose_t.y();
        tempt[2]=frames[ulong(i)].inv_V3d_pose_t.z();
        Rposeofframes.insert(make_pair(i,tempR));
        tposeofframes.insert(make_pair(i,tempt));
        //cout<<Rposeofframes[i][0]<<Rposeofframes[i][1]<<Rposeofframes[i][2]<<endl;
        problem.AddParameterBlock(Rposeofframes[i],4,local_parameterization);
        problem.AddParameterBlock(tposeofframes[i],3);
        //set the first frame pose to constant,set the transpose from l to last to constant(1).
        if(i==l){
            problem.SetParameterBlockConstant(Rposeofframes[i]);
            problem.SetParameterBlockConstant(tposeofframes[i]);
        }
        if(i==endframe){
            //problem.SetParameterBlockConstant(Rposeofframes[i]);
            problem.SetParameterBlockConstant(tposeofframes[i]);
        }
    }
    int pointstart=frames[ulong(statrframe)].featurePointscoords_norm.front().first;
    int pointend=frames[ulong(endframe)].featurePointscoords_norm.back().first;
    for(int i=pointstart;i<=pointend;i++){
        if(featurePoints[ulong(i)].triangulated==false)
            continue;
        //add parameter block.
        double* temppoint = featurePoints[ulong(i)].point3d_forba ;
        temppoint[0]=featurePoints[ulong(i)].point3d_Eigen.x();
        temppoint[1]=featurePoints[ulong(i)].point3d_Eigen.y();
        temppoint[2]=featurePoints[ulong(i)].point3d_Eigen.z();
        coordinateofpoints.insert(make_pair(i,temppoint));
        for(size_t j=0;j<featurePoints[ulong(i)].inWithcframes.size();j++){
            int frameindex=featurePoints[ulong(i)].inWithcframes[ulong(j)];
            if(frames[ulong(frameindex)].posesolved==false)
                continue;
            ceres::CostFunction* costfunction=ReprojectError::Create(
                        static_cast<double>(featurePoints[ulong(i)].normpoints[j].second.x),
                        static_cast<double>(featurePoints[ulong(i)].normpoints[j].second.y));
            problem.AddResidualBlock(costfunction,nullptr,
                                     Rposeofframes[frameindex],
                                     tposeofframes[frameindex],
                                     coordinateofpoints[i]);
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds=100;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    if(summary.termination_type==ceres::CONVERGENCE||summary.final_cost<5e-3){
        cout<<summary.BriefReport()<<endl;
        cout<<"ba converge."<<endl;
    }else{
        cout<<"ba failed"<<endl;
        //return false;
    }

    //update pose
    for(int i=statrframe;i<=endframe;i++){
        Quaterniond q;
        q.w()=frames[ulong(i)].q_pose_forba[0];
        q.x()=frames[ulong(i)].q_pose_forba[1];
        q.y()=frames[ulong(i)].q_pose_forba[2];
        q.z()=frames[ulong(i)].q_pose_forba[3];
        q=q.inverse();
        Matrix3d r33d(q);
        Vector3d v3d(frames[ulong(i)].t_pose_forba[0],frames[ulong(i)].t_pose_forba[1],
                frames[ulong(i)].t_pose_forba[2]);
        v3d=-r33d*v3d;
        frames[ulong(i)].setpose(r33d,v3d);
    }

    //update points
    for(int i=pointstart;i<=pointend;i++){
        if(featurePoints[ulong(i)].triangulated==false)
            continue;
        featurePoints[ulong(i)].point3d_Eigen.x()=featurePoints[ulong(i)].point3d_forba[0];
        featurePoints[ulong(i)].point3d_Eigen.y()=featurePoints[ulong(i)].point3d_forba[1];
        featurePoints[ulong(i)].point3d_Eigen.z()=featurePoints[ulong(i)].point3d_forba[2];
        featurePoints[ulong(i)].point3d_Point3f.x=static_cast<float>(featurePoints[ulong(i)].point3d_forba[0]);
        featurePoints[ulong(i)].point3d_Point3f.y=static_cast<float>(featurePoints[ulong(i)].point3d_forba[1]);
        featurePoints[ulong(i)].point3d_Point3f.z=static_cast<float>(featurePoints[ulong(i)].point3d_forba[2]);

    }

    return true;
}

void VioOdometry::writeFramepose(string filepath)
{
    ofstream file;
    file.open(filepath);

    for(size_t i=0;i<frames.size();i++){
        if(frames[i].posesolved){

            file<<left<<setw(4)<<frames[i].id<<"  "
                <<left<<setw(12)<<frames[i].q_pose_R.w()<<"  "
                <<left<<setw(12)<<frames[i].q_pose_R.x()<<"  "
                <<left<<setw(12)<<frames[i].q_pose_R.y()<<"  "
                <<left<<setw(12)<<frames[i].q_pose_R.z()<<"  "
                <<left<<setw(12)<<frames[i].V3d_pose_t.x()<<"  "
                <<left<<setw(12)<<frames[i].V3d_pose_t.y()<<"  "
                <<left<<setw(12)<<frames[i].V3d_pose_t.z()<<"  "
                <<"\n";
        }
    }

    file.close();
}

void VioOdometry::triangulatePointByMinsSquare(Matrix<double,4,4> T1, Matrix<double,4,4> T2, vector<Vector3d> pts1, vector<Vector3d> pts2, vector<int> indexs)
{
    Matrix<double,4,4> T12=T1.inverse()*T2;
    Matrix3d R=T12.block<3,3>(0,0);
    Vector3d t=T12.block<3,1>(0,3);
    for(size_t i=0;i<indexs.size();i++){
        MatrixXd a=pts1[i].transpose()*pts1[i];
        MatrixXd b=pts1[i].transpose()*R*pts2[i];
        MatrixXd c=pts1[i].transpose()*t;
        MatrixXd d=(R*pts2[i]).transpose()*R*pts2[i];
        MatrixXd e=t.transpose()*R*pts2[i];
        double lh,lo,li;
        lh=a(0,0)*d(0,0)-b(0,0)*b(0,0);
        lo=d(0,0)*c(0,0)-b(0,0)*e(0,0);
        li=b(0,0)*c(0,0)-a(0,0)*e(0,0);
        double s1,s2;
        s1=lo/lh;
        s2=li/lh;
        //cout<<s1<<endl;
        //cout<<s2<<endl;
        Vector3d p1=s1*pts1[i];
        Vector3d p2=R*s2*pts2[i]+t;
        MatrixXd err=(p1-p2).transpose()*(p1-p2);
        if(err(0,0)<0.001){
        }
    }
}


























