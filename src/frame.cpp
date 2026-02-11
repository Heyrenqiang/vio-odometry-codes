// frame.cpp - 帧数据结构的实现
// Frame类封装了单帧图像的所有信息：
// - 位姿（旋转和平移）
// - 特征点观测
// - IMU预积分数据
// 支持Eigen和OpenCV两种格式的位姿存储

#include "frame.h"

// 默认构造函数
Frame::Frame(){

}


/**
 * @brief 添加单个特征点到当前帧
 * @param featurepoint 特征点对象
 * 
 * 更新当前帧的特征点列表，包括：
 * - 特征点ID列表
 * - 像素坐标（原始和归一化）
 */
void Frame::addFeaturePoints(FeaturePoints featurepoint)
{
    features.push_back(featurepoint.pointid);
    featurePointscoords.push_back(make_pair(featurepoint.pointid,featurepoint.pixelpoints.back().second));
    featurePointscoords_norm.push_back(make_pair(featurepoint.pointid,featurepoint.normpoints.back().second));
    featurePointscoords_norm_2d.push_back(make_pair(featurepoint.pointid,featurepoint.normpoints2d.back().second));
}

/**
 * @brief 添加跟踪到的特征点（指向前一帧）
 * @param id 特征点ID
 * @param p 特征点坐标
 * 
 * 建立当前帧与前一帧的特征点对应关系
 */
void Frame::addTrackedFeaturePointstoprevious(int id,Point2f p)
{
    trackedFeaturePointstoprevious.push_back(make_pair(id,p));
    trackedPointstoprev.push_back(p);
}

/**
 * @brief 设置帧的位姿
 * @param R 旋转矩阵（Eigen 3x3）
 * @param t 平移向量（Eigen 3x1）
 * 
 * 同时更新多种位姿表示：
 * - Eigen格式：旋转矩阵、平移向量、四元数
 * - OpenCV格式：Mat矩阵（用于OpenCV函数）
 * - 4x4变换矩阵（用于三角化）
 * - 逆变换（世界到相机）
 */
void Frame::setpose(Matrix3d &R,Vector3d &t)
{
    // Eigen格式存储
    M3d_pose_R=R;
    V3d_pose_t=t;
    q_pose_R=Quaterniond(M3d_pose_R);  // 旋转矩阵转四元数
    inv_q_pose_R=q_pose_R.inverse();    // 逆旋转
    
    // 构建4x4变换矩阵
    M34_pose_Rt.block<3,3>(0,0)=M3d_pose_R;
    M34_pose_Rt.block<3,1>(0,3)=V3d_pose_t;
    M4d_pose_T.block<3,4>(0,0)=M34_pose_Rt;
    M4d_pose_T.block<1,4>(3,0)=Matrix<double,1,4>(0,0,0,1);
    
    // 转换为OpenCV格式
    eigen2cv(M3d_pose_R,Mat33_pose_R);
    eigen2cv(V3d_pose_t,Mat31_pose_t);
    eigen2cv(M4d_pose_T,Mat44_pose_T);
    
    // 构建3x4投影矩阵
    Mat34_pose_Rt=(Mat_<double>(3,4)<<
                Mat33_pose_R.at<double>(0,0),Mat33_pose_R.at<double>(0,1),Mat33_pose_R.at<double>(0,2),Mat31_pose_t.at<double>(0,0),
                Mat33_pose_R.at<double>(1,0),Mat33_pose_R.at<double>(1,1),Mat33_pose_R.at<double>(1,2),Mat31_pose_t.at<double>(1,0),
                Mat33_pose_R.at<double>(2,0),Mat33_pose_R.at<double>(2,1),Mat33_pose_R.at<double>(2,2),Mat31_pose_t.at<double>(2,0));
    
    // 构建4x4变换矩阵
    Mat44_pose_T=(Mat_<double>(4,4)<<
                Mat33_pose_R.at<double>(0,0),Mat33_pose_R.at<double>(0,1),Mat33_pose_R.at<double>(0,2),Mat31_pose_t.at<double>(0,0),
                Mat33_pose_R.at<double>(1,0),Mat33_pose_R.at<double>(1,1),Mat33_pose_R.at<double>(1,2),Mat31_pose_t.at<double>(1,0),
                Mat33_pose_R.at<double>(2,0),Mat33_pose_R.at<double>(2,1),Mat33_pose_R.at<double>(2,2),Mat31_pose_t.at<double>(2,0),
                0,0,0,1);
    
    // 计算逆变换（世界坐标系到相机坐标系）
    inv_M4d_pose_T=M4d_pose_T.inverse();
    inv_M34_pose_Rt=inv_M4d_pose_T.block<3,4>(0,0);
    inv_M3d_pose_R=inv_M4d_pose_T.block<3,3>(0,0);
    inv_V3d_pose_t=inv_M4d_pose_T.block<3,1>(0,3);

    // 转换逆变换为OpenCV格式
    eigen2cv(inv_M4d_pose_T,inv_Mat44_pose_T);
    eigen2cv(inv_M34_pose_Rt,inv_Mat34_pose_Rt);
    eigen2cv(inv_M3d_pose_R,inv_Mat33_pose_R);
    eigen2cv(inv_V3d_pose_t,inv_Mat31_pose_t);
}

/**
 * @brief 添加IMU预积分信息
 * @param slic_accs_ 加速度计数据序列
 * @param slic_omigas_ 陀螺仪数据序列
 * @param slic_imustamps_ IMU时间戳序列
 * 
 * 初始化IMU预积分相关的状态：
 * - 检查是否有足够的IMU数据
 * - 初始化偏置（默认为零）
 * - 初始化预积分变量（位移、旋转、速度）
 * - 初始化Jacobian和协方差
 */
void Frame::addPreInformation(vector<Vector3d> slic_accs_,vector<Vector3d> slic_omigas_,vector<double> slic_imustamps_)
{
    // 检查是否有足够的IMU数据（期望11个样本）
    if(slic_accs_.size()==11)
        hasenoughimudata=true;
    else
        hasenoughimudata=false;
    
    // 存储IMU数据
    slic_accs=slic_accs_;
    slic_omigas=slic_omigas_;
    slic_imustamps=slic_imustamps_;

    // 初始化偏置为零
    acc_bias=Vector3d::Zero();
    gyr_bias=Vector3d::Zero();
    
    // 初始化预积分变量
    pre_integration_pose_gama=Quaterniond::Identity();  // 旋转（四元数）
    pre_integration_pose_alpha=Vector3d::Zero();        // 位移
    pre_integration_pose_beta=Vector3d::Zero();         // 速度

    // 初始化Jacobian和协方差
    jacobian=MatrixXd::Identity(15,15);
    covariance=MatrixXd::Zero(12,12);

    // 初始化偏置变化量为零
    delta_gyr_bias=Vector3d::Zero();
    delta_acc_bias=Vector3d::Zero();

}

/**
 * @brief 添加跟踪到的特征点（指向后一帧）
 * @param id 特征点ID
 * @param p 特征点坐标
 * 
 * 建立当前帧与后一帧的特征点对应关系
 */
void Frame::addTrackedFeaturePointstonext(int id,Point2f p)
{
    trackedFeaturePointstonext.push_back(make_pair(id,p));
    trackedPointstonext.push_back(p);
}

/**
 * @brief 带参数的构造函数
 * @param timestamp 时间戳
 * @param id 帧ID
 * 
 * 初始化帧对象，设置位姿未求解状态
 */
Frame::Frame( const double timestamp, const int id):
   id(id), timestamp(timestamp)
{
    posesolved=false;  // 初始状态：位姿未求解
}
