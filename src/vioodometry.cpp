// vioodometry.cpp - 视觉惯性里程计主实现文件
// 包含VIO系统的核心功能：特征跟踪、初始化、BA优化、IMU预积分等

#include "vioodometry.h"
#include"imuvisualalian.h"

// 默认构造函数
VioOdometry::VioOdometry()
{

}

/**
 * @brief 主跟踪函数 - 处理每一帧图像和IMU数据
 * @param img 输入图像（灰度图）
 * @param timestamp 图像时间戳
 * @param slic_accs 当前帧与上一帧之间的加速度计数据
 * @param slic_omigas 当前帧与上一帧之间的陀螺仪数据
 * @param slic_imustamps IMU数据时间戳
 * @param frameid 当前帧ID
 * 
 * 功能流程：
 * 1. 创建帧对象并存储IMU数据
 * 2. 执行IMU预积分
 * 3. 特征点跟踪（光流法）
 * 4. 判断是否需要初始化
 * 5. 执行SFM初始化、BA优化、视觉IMU对齐
 */
void VioOdometry::Track(const Mat& img,double timestamp,vector<Vector3d> slic_accs,vector<Vector3d> slic_omigas,
                        vector<double> slic_imustamps,int frameid)
{
    curframeid=frameid;
    cout.setf(ios::fixed);
    cout<<setprecision(9);
    
    // 步骤1: 创建帧对象并用帧ID标记
    Frame frame(timestamp,frameid);
    frame.addPreInformation(slic_accs,slic_omigas,slic_imustamps);
    frames.push_back(frame);

    // 步骤2: IMU预积分 - 使用 midpoint 或普通积分方法
    if(MIDINTEGRATION)
        ImuMidPreintegration(frameid);  // 中点积分（精度更高）
    else
        ImuPreintegration(frameid);      // 普通积分

    // 步骤3: 特征点跟踪
    // 跟踪策略：
    // - 如果是第一帧，检测新的特征点
    // - 否则，使用LK光流跟踪上一帧的特征点
    vector<int> pointsinprevframes;
    if(frameid==0){
        pointsinprevframes.clear();  // 第一帧没有前一帧的点
    }else{
       pointsinprevframes.clear();
        // 获取前一帧的所有特征点ID
        for(size_t i=0;i<frames[frames.size()-2].featurePointscoords_norm.size();i++){
            pointsinprevframes.push_back(
                        frames[frames.size()-2].featurePointscoords_norm[i].first);
        }
    }
    
    vector<Point2f> newaddedpoints;  // 存储新检测的特征点
    vector<pair<int,pair<Point2f,Point2f>>> trackedpointsincomingframes;  // 存储跟踪到的点
    // 执行特征点跟踪
    feature_Track.trackFeaturePoints2(img,frameid,pointsinprevframes,newaddedpoints,trackedpointsincomingframes);
    
    // 处理跟踪到的特征点：更新特征点历史并添加到当前帧
    for(size_t i=0;i<trackedpointsincomingframes.size();i++){
        ulong index=ulong(trackedpointsincomingframes[i].first);
        Point2f p=trackedpointsincomingframes[i].second.first;      // 去畸变后的坐标
        Point2f raw_p=trackedpointsincomingframes[i].second.second; // 原始像素坐标
        featurePoints[index].addFrameThatsaw(frameid,p,raw_p);
        frames[ulong(frameid)].addFeaturePoints(featurePoints[index]);
    }
    
    // 处理新检测的特征点：创建新的特征点对象
    for(size_t i=0;i<newaddedpoints.size();i++){
        FeaturePoints featurepoint(newaddedpoints[i],frameid);
        featurePoints.push_back(featurepoint);
        frames[ulong(frameid)].addFeaturePoints(featurepoint);
    }
    
    // 可视化：显示当前帧检测到的特征点（如果开启SHOWIMG）
    if(SHOWIMG){
        Mat imgshow=img.clone();
        for(auto kp: frame.featurePointscoords){
            circle(imgshow,kp.second,4,Scalar(0,240,0),3);
        }
        imshow("corners",imgshow);
        waitKey(0);
    }
    
    // 步骤4: 判断是否需要开始初始化
    // 当帧ID超过STARTFRAME时，开始初始化流程
    if(frameid>STARTFRAME){
        StartInitial_flag=true;
    }else{
        StartInitial_flag=false;
    }
    
    // 步骤5: 执行初始化流程
    if(StartInitial_flag){
        Mat relative_R,relative_t;
        int l;
        // 判断当前帧与前面帧是否有足够的视差和对匹配
        if(judgeInitcondition(frameid,relative_R,relative_t,l)){

            // 5.1 执行SFM（Structure from Motion）恢复初始帧位姿和地图点
            recoverPoseofSFM(l,frameid);
            writeFramepose("../sfm-project/data/trajectory.txt");  // 保存SFM结果
            
            // 5.2 执行BA（Bundle Adjustment）优化
            if(!initBaOptimization(frameid-NUMOFFRAMESTOINIT+1,frameid,l)){
                cout<<"init ba failed."<<endl;
            }
            RecoverPoseOfadjacentFrame();  // 恢复相邻帧位姿
            writeFramepose("../sfm-project/data/trajectoryafterba.txt");

            // 5.3 视觉IMU对齐 - 估计陀螺仪偏置、重力方向、速度、尺度
            VisualImuAlian();

            // 5.4 可视化轨迹和地图点
            vector<string> paths;
            paths.push_back("../sfm-project/data/trajectory.txt");
            paths.push_back("../sfm-project/data/trajectoryafterba.txt");
            paths.push_back("../sfm-project/data/groundtruth.txt");
            DataReading::showTrajectory(paths,0,10000);

            // 准备3D地图点用于可视化
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
           // 不满足初始化条件，继续收集帧
        }
    }
}

// 求解初始帧位姿（2D-2D匹配）
void VioOdometry::solvePoseofframesatbegining()
{
    size_t l=frames.size()-size_t(NUMOFFRAMESTOINIT);
    size_t lend=frames.size();
    for(size_t i=l;i<lend-1;i++){
        Mat R,t;
        // 使用本质矩阵求解相对位姿
        solvePose.solvePoseby2d2d(frames[i].trackedPointstonext,frames[i+1].trackedPointstoprev,R,t);

    }
}

void VioOdometry::trackFeatures()
{

}

/**
 * @brief 初始化噪声参数和偏置
 * 设置加速度计和陀螺仪的噪声协方差矩阵
 */
void VioOdometry::Initialparameters()
{
    // 初始化12x12噪声矩阵（用于普通预积分）
    noise=MatrixXd::Zero(12,12);
    noise.block<3,3>(0,0)=ACC_N*ACC_N*Matrix3d::Identity();    // 加速度噪声
    noise.block<3,3>(3,3)=GYR_N*GYR_N*Matrix3d::Identity();    // 陀螺仪噪声
    noise.block<3,3>(6,6)=ACC_B_N*ACC_B_N*Matrix3d::Identity(); // 加速度偏置随机游走
    noise.block<3,3>(9,9)=GYR_B_N*GYR_B_N*Matrix3d::Identity(); // 陀螺仪偏置随机游走
    
    // 初始化18x18噪声矩阵（用于midpoint积分）
    noise_18_18=MatrixXd::Zero(18,18);
    noise_18_18.block<3,3>(0,0)=ACC_N*ACC_N*Matrix3d::Identity();
    noise_18_18.block<3,3>(3,3)=GYR_N*GYR_N*Matrix3d::Identity();
    noise_18_18.block<3,3>(6,6)=ACC_N*ACC_N*Matrix3d::Identity();
    noise_18_18.block<3,3>(9,9)=GYR_N*GYR_N*Matrix3d::Identity();
    noise_18_18.block<3,3>(12,12)=ACC_B_N*ACC_B_N*Matrix3d::Identity();
    noise_18_18.block<3,3>(15,15)=GYR_B_N*GYR_B_N*Matrix3d::Identity();
    
    // 初始化偏置为零
    gyr_bias=Vector3d::Zero();
    acc_bias=Vector3d::Zero();
}

/**
 * @brief 获取两帧之间的匹配特征点
 * @param frontframe 前帧ID
 * @param backframe 后帧ID
 * @param matchedfeatures 输出的匹配点对
 * 
 * 查找在两帧中都出现的特征点，返回它们的像素坐标对
 */
void VioOdometry::getMatchedFeatures(int frontframe, int backframe, vector<pair<Point2f, Point2f> > &matchedfeatures)
{
    int p_star=frames[ulong(frontframe)].featurePointscoords[0].first;
    int p_end=frames[ulong(backframe)].featurePointscoords.back().first;
    for(int i=p_star;i<=p_end;i++){
        // 检查特征点是否在两帧中都出现
        if(featurePoints[ulong(i)].inWithcframes.front()<=int(frontframe)&&featurePoints[ulong(i)].inWithcframes.back()>=int(backframe)){
            Point2f p1;
            // 查找特征点在前帧中的坐标
            for(size_t k=0;k<featurePoints[ulong(i)].pixelpoints.size();k++){
                if(featurePoints[ulong(i)].pixelpoints[k].first==frontframe){
                    p1=featurePoints[ulong(i)].pixelpoints[k].second;
                    break;
                }
            }
            // 后帧中使用最新坐标
            Point2f p2=featurePoints[ulong(i)].pixelpoints.back().second;
            matchedfeatures.push_back(make_pair(p1,p2));
        }
    }
}

/**
 * @brief 计算视差（parallax）
 * @param parallax 输出的平均视差值
 * @param matchedfeatures 匹配的特征点对
 * 
 * 视差是判断初始化质量的重要指标，视差越大说明运动越大
 */
void VioOdometry::getParallax(double &parallax, vector<pair<Point2f, Point2f> > matchedfeatures)
{
    double sumparallax=0;
    for(size_t i=0;i<matchedfeatures.size();i++){
        Vector2d p1(matchedfeatures[i].first.x,matchedfeatures[i].first.y);
        Vector2d p2(matchedfeatures[i].second.x,matchedfeatures[i].second.y);
        double temp=(p1-p2).norm();  // 计算欧氏距离
        sumparallax=sumparallax+temp;
    }
    // 乘以焦距转换为实际视差
    parallax=sumparallax/matchedfeatures.size()*460;
}

/**
 * @brief 判断初始化条件
 * @param curframeid 当前帧ID
 * @param relative_R 输出的相对旋转（引用）
 * @param relative_t 输出的相对平移（引用）
 * @param l 输出的参考帧ID
 * @return 是否满足初始化条件
 * 
 * 条件：
 * 1. 有足够的视差（>30像素）
 * 2. 能通过本质矩阵求解相对位姿
 */
bool VioOdometry::judgeInitcondition(int curframeid,Mat &relative_R,Mat &relative_t,int &l)
{
    for(int i=curframeid-NUMOFFRAMESTOINIT+1;i<curframeid;i++){

        vector<pair<Point2f, Point2f> > matchedfeatures;
        getMatchedFeatures(i,curframeid,matchedfeatures);
        double parallax;
        getParallax(parallax,matchedfeatures);
        // 检查视差和位姿解算是否成功
        if(parallax>30&&solvePose.solvePoseby2d2d(matchedfeatures,relative_R,relative_t)){
            l=i;  // 记录参考帧
            frames[ulong(l)].posesolved=true;
            frames[ulong(curframeid)].posesolved=true;
            // 设置世界坐标系：l帧为世界坐标系原点
            Mat temp_R=(Mat_<double>(3,3)<<1,0,0,0,1,0,0,0,1);
            Mat temp_t=(Mat_<double>(3,1)<<0,0,0);
            Mat relative_R_t=relative_R.inv();
            Mat relative_t_t=-relative_R_t*relative_t;
            Matrix3d Rl,Rcur;
            Vector3d tl,tcur;
            // 转换OpenCV矩阵到Eigen
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
            return true;
        }
    }
    return false;
}

/**
 * @brief 执行Structure from Motion恢复初始位姿和地图点
 * @param l 参考帧ID
 * @param frameid 当前帧ID
 * @return 是否成功
 * 
 * 流程：
 * 1. 从l帧到当前帧，使用PnP求解每帧位姿
 * 2. 三角化地图点
 * 3. 从l帧向前扩展
 * 4. BA优化
 */
bool VioOdometry::recoverPoseofSFM(int l, int frameid)
{
    // 前向传播：从l帧到当前帧
    for(int i=l;i<frameid;i++){
        if(i>l){
            if(!solveFrameByPnp(i)){  // PnP求解位姿
                return false;
            }
        }
        triangulateTwoFrames(i,frameid);  // 三角化地图点
        if(i==l){
            if(!optimizatetwoFrames(i,frameid)){  // 两帧BA优化
                return false;
            }
        }
    }
    // 补充三角化
    for(int i=l+1;i<frameid;i++){
        triangulateTwoFrames(l,i);
    }

    // 后向传播：从l帧向前扩展
    for(int i=l-1;i>frameid-NUMOFFRAMESTOINIT;i--){
        if(!solveFrameByPnp(i)){
            return false;
        }
        triangulateTwoFrames(i,l);
    }
    // 相邻帧三角化
    for(int i=frameid-NUMOFFRAMESTOINIT+1;i<frameid;i++){
        triangulateTwoFrames(i,i+1);
    }
    cout<<"SFM init succed!"<<endl;
    return true;
}

/**
 * @brief 使用PnP求解单帧位姿
 * @param l 待求解的帧ID
 * @return 是否成功
 * 
 * 使用已三角化的3D点通过PnP求解相机位姿
 */
bool VioOdometry::solveFrameByPnp(int l)
{

    vector<Point3f> pts_3d;       // 3D地图点
    vector<Point2f> pts_2d;       // 2D像素点
    vector<Point3f> pts_3d_norm;  // 归一化坐标
    
    // 收集当前帧中已三角化的特征点
    for(size_t i=0;i<frames[ulong(l)].featurePointscoords.size();i++){
        int index=frames[ulong(l)].featurePointscoords[i].first;
        if(featurePoints[ulong(index)].triangulated){
            pts_2d.push_back(frames[ulong(l)].featurePointscoords_norm_2d[i].second);
            pts_3d_norm.push_back(frames[ulong(l)].featurePointscoords_norm[i].second);
            pts_3d.push_back(featurePoints[ulong(index)].point3d_Point3f);
        }
    }
    // 检查特征点数量是否足够
    if(pts_2d.size()<15){
        cout<<"unstable feature tracking"<<endl;
        if(pts_2d.size()<10){
            return false;
        }
    }
    Mat r,revc,t,D,tmp_r;

    // 使用前一帧或后一帧的位姿作为初始值
    if(l>0&&frames[ulong(l-1)].posesolved){
        tmp_r=frames[ulong(l-1)].Mat33_pose_R;
        Rodrigues(tmp_r,revc);
        t=frames[ulong(l-1)].Mat31_pose_t;
    }else if(frames[ulong(l+1)].posesolved){
        tmp_r=frames[ulong(l+1)].Mat33_pose_R;
        Rodrigues(tmp_r,revc);
        t=frames[ulong(l+1)].Mat31_pose_t;
    }

    Mat K=(Mat_<double>(3,3)<<1,0,0,0,1,0,0,0,1);  // 单位矩阵（使用归一化坐标）
    bool pnpsucc;
    // 调用OpenCV PnP求解
    pnpsucc=solvePnP(pts_3d,pts_2d,K,D,revc,t,1);
    if(!pnpsucc){
        return false;
    }
    Rodrigues(revc,r);
    Matrix3d pose_R;
    Vector3d pose_t;
    cv2eigen(r,pose_R);
    cv2eigen(t,pose_t);
    // 转换为相机到世界的变换
    Matrix3d inv_pose_R=pose_R.inverse();
    Vector3d inv_pose_t=-inv_pose_R*pose_t;
    frames[ulong(l)].setpose(inv_pose_R,inv_pose_t);
    frames[ulong(l)].posesolved=true;
    return true;

}

/**
 * @brief 两帧三角化地图点
 * @param n 第一帧ID
 * @param m 第二帧ID
 * 
 * 使用OpenCV的triangulatePoints函数进行三角化
 */
void VioOdometry::triangulateTwoFrames(int n, int m)
{
    // 确定特征点范围
    int p_star=frames[ulong(n)].featurePointscoords[0].first;
    int p_end=frames[ulong(m)].featurePointscoords.back().first;
    vector<Vector2d> pn,pm;
    vector<Point2f> pnf,pmf;
    vector<int> indexs;
    
    // 查找在两帧中都出现的特征点
    for(int i=p_star;i<=p_end;i++){
        if(featurePoints[ulong(i)].triangulated)
            continue;  // 跳过已三角化的点
        bool has_n=false,has_m=false;
        Point2f tempn,tempm;
        int tempindexn=-1,tempindexm = -1;
        // 检查特征点在两帧中的索引
        for(size_t j=0;j<featurePoints[ulong(i)].inWithcframes.size();j++){
            if(featurePoints[ulong(i)].inWithcframes[j]==n){
                tempn=featurePoints[ulong(i)].normpoints2d[j].second;
                tempindexn=int(j);
                has_n=true;
            }
            if(featurePoints[ulong(i)].inWithcframes[j]==m){
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
    // 执行三角化
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
        // OpenCV三角化（输出齐次坐标）
        triangulatePoints(frames[ulong(n)].inv_Mat34_pose_Rt,frames[ulong(m)].inv_Mat34_pose_Rt,pnf,pmf,p4d);
        for(int i=0;i<p4d.cols;i++){
            Mat x=p4d.col(i);
            MatrixXd xtemp;
            cv2eigen(x,xtemp);
            xtemp=xtemp/(xtemp(3,0));  // 归一化齐次坐标
            Point3d p(xtemp(0,0),
                      xtemp(1,0),
                      xtemp(2,0));
            // 保存三角化结果
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

/**
 * @brief 两帧BA优化
 * @param first 第一帧ID
 * @param second 第二帧ID
 * @return 是否成功
 * 
 * 使用Ceres Solver进行Bundle Adjustment优化
 */
bool VioOdometry::optimizatetwoFrames(int first,int second){
    // 创建Ceres优化问题
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization=new ceres::QuaternionParameterization();
    map<int,double*> Rposeofframes;  // 存储各帧的旋转四元数
    map<int,double*> tposeofframes;  // 存储各帧的平移
    map<int,double*>  coordinateofpoints;  // 存储地图点坐标
    double* tempR ;
    double* tempt ;
    
    // 添加第一帧参数块
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
    
    // 添加第二帧参数块
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
    
    // 添加参数块到优化问题
    problem.AddParameterBlock(Rposeofframes[first],4,local_parameterization);
    problem.AddParameterBlock(tposeofframes[first],3);
    problem.AddParameterBlock(Rposeofframes[second],4,local_parameterization);
    problem.AddParameterBlock(tposeofframes[second],3);
    // 固定第一帧作为参考
    problem.SetParameterBlockConstant(Rposeofframes[first]);
    problem.SetParameterBlockConstant(tposeofframes[first]);

    // 添加地图点参数和重投影误差
    int pointstart=frames[ulong(first)].featurePointscoords_norm.front().first;
    int pointend=frames[ulong(second)].featurePointscoords_norm.back().first;
    for(int i=pointstart;i<=pointend;i++){
        if(featurePoints[ulong(i)].triangulated==false)
            continue;
        double* temppoint = featurePoints[ulong(i)].point3d_forba ;
        temppoint[0]=featurePoints[ulong(i)].point3d_Eigen.x();
        temppoint[1]=featurePoints[ulong(i)].point3d_Eigen.y();
        temppoint[2]=featurePoints[ulong(i)].point3d_Eigen.z();
        coordinateofpoints.insert(make_pair(i,temppoint));
        problem.AddParameterBlock(coordinateofpoints[i],3);
        
        // 添加所有观测的重投影误差
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
    
    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds=100;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    // 检查优化结果
    if(summary.termination_type==ceres::CONVERGENCE||summary.final_cost<5e-3){
        cout<<summary.BriefReport()<<endl;
        cout<<"ba converge."<<endl;
    }else{
        cout<<"ba failed"<<endl;
    }

    // 更新优化后的位姿（从世界到相机转换回相机到世界）
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

    // 更新优化后的地图点
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

/**
 * @brief 使用PnP恢复相邻帧位姿
 * 从已三角化的帧向前后扩展
 */
void VioOdometry::RecoverPoseOfadjacentFrame()
{
    for(size_t i=frames.size()-ulong(NUMOFFRAMESTOINIT)-1;i>0;i--){
        int numofmatchfeatures=0;
        // 统计已三角化的特征点数量
        for(size_t j=0;j<frames[i].features.size();j++){
            if(featurePoints[ulong(frames[i].features[j])].triangulated){
                numofmatchfeatures++;
            }
        }
        // 如果特征点足够，使用PnP求解
        if(numofmatchfeatures>15){
               solveFrameByPnp(int(i));
        }
        else{
            return;
        }
    }
    // 处理第一帧
    int numofmatchfeatures=0;
    for(size_t i=0;i<frames.front().features.size();i++){
        if(featurePoints[ulong(frames.front().features[i])].triangulated){
            numofmatchfeatures++;
        }
        if(numofmatchfeatures>15){
            solveFrameByPnp(0);
        }
    }
}

/**
 * @brief 初始化BA优化 - 优化所有初始帧和地图点
 * @param statrframe 起始帧ID
 * @param endframe 结束帧ID
 * @param l 参考帧ID
 * @return 是否成功
 */
bool VioOdometry::initBaOptimization(int statrframe, int endframe,int l)
{

    // 创建Ceres优化问题
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization=new ceres::QuaternionParameterization();
    map<int,double*> Rposeofframes;
    map<int,double*> tposeofframes;
    map<int,double*>  coordinateofpoints;

    // 添加所有帧的参数块
    for(int i=statrframe;i<=endframe;i++){
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
        
        problem.AddParameterBlock(Rposeofframes[i],4,local_parameterization);
        problem.AddParameterBlock(tposeofframes[i],3);
        // 固定参考帧和最后一帧的平移
        if(i==l){
            problem.SetParameterBlockConstant(Rposeofframes[i]);
            problem.SetParameterBlockConstant(tposeofframes[i]);
        }
        if(i==endframe){
            problem.SetParameterBlockConstant(tposeofframes[i]);
        }
    }
    
    // 添加地图点参数和重投影误差
    int pointstart=frames[ulong(statrframe)].featurePointscoords_norm.front().first;
    int pointend=frames[ulong(endframe)].featurePointscoords_norm.back().first;
    for(int i=pointstart;i<=pointend;i++){
        if(featurePoints[ulong(i)].triangulated==false)
            continue;
        double* temppoint = featurePoints[ulong(i)].point3d_forba ;
        temppoint[0]=featurePoints[ulong(i)].point3d_Eigen.x();
        temppoint[1]=featurePoints[ulong(i)].point3d_Eigen.y();
        temppoint[2]=featurePoints[ulong(i)].point3d_Eigen.z();
        coordinateofpoints.insert(make_pair(i,temppoint));
        
        // 添加该特征点在所有帧中的观测
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
    
    // 求解
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
    }

    // 更新所有帧的位姿
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

    // 更新所有地图点
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

/**
 * @brief 将帧位姿写入文件
 * @param filepath 输出文件路径
 * 
 * 格式：frame_id qw qx qy qz tx ty tz
 */
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

/**
 * @brief 使用最小二乘三角化（备用方法）
 * @param T1 第一帧位姿
 * @param T2 第二帧位姿
 * @param pts1 第一帧中的归一化坐标
 * @param pts2 第二帧中的归一化坐标
 * @param indexs 特征点索引
 */
void VioOdometry::triangulatePointByMinsSquare(Matrix<double,4,4> T1, Matrix<double,4,4> T2, vector<Vector3d> pts1, vector<Vector3d> pts2, vector<int> indexs)
{
    Matrix<double,4,4> T12=T1.inverse()*T2;  // 相对位姿
    Matrix3d R=T12.block<3,3>(0,0);
    Vector3d t=T12.block<3,1>(0,3);
    for(size_t i=0;i<indexs.size();i++){
        // 最小二乘求解深度
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
        Vector3d p1=s1*pts1[i];
        Vector3d p2=R*s2*pts2[i]+t;
        MatrixXd err=(p1-p2).transpose()*(p1-p2);
        if(err(0,0)<0.001){
            // 重投影误差小，三角化成功
        }
    }
}





































