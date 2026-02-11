// parameters.cpp - 系统参数管理
// 负责从YAML配置文件读取相机、IMU、特征跟踪等参数
// 所有参数作为全局变量定义，供其他模块访问

#include "parameters.h"

// ===== 全局参数变量定义 =====

// ROS主题名称
std::string IMAGE_TOPIC;       // 图像话题
std::string IMU_TOPIC;         // IMU话题
std::vector<std::string> CAM_NAMES;  // 相机名称列表
std::string FISHEYE_MASK;      // 鱼眼相机掩膜路径

// 特征跟踪参数
int MAX_CNT;           // 最大特征点数量
double MIN_DIST;       // 特征点间最小距离
int WINDOW_SIZE;       // 滑动窗口大小
int FREQ;              // 跟踪结果发布频率
double F_THRESHOLD;    // RANSAC阈值
int SHOW_TRACK;        // 是否显示跟踪图像
int STEREO_TRACK;      // 是否使用双目跟踪
int EQUALIZE;          // 是否使用直方图均衡
int ROW, COL;          // 图像尺寸
int FOCAL_LENGTH;      // 焦距
int FISHEYE;           // 是否使用鱼眼相机
bool PUB_THIS_FRAME;   // 是否发布当前帧

// 初始化参数
int NUMOFFRAMESTOINIT; // 初始化所需帧数
int STARTFRAME;        // 开始初始化的帧ID
bool SHOWIMG;          // 是否显示图像
bool IFSHOWPOINTS;     // 是否显示点云

// IMU噪声参数
double ACC_N,GYR_N;      // 加速度计和陀螺仪测量噪声
double ACC_B_N,GYR_B_N;  // 偏置随机游走噪声

// 相机-IMU外参
MatrixXd T_BS;    // 相机到IMU的4x4变换矩阵
MatrixXd T_BS_R;  // 旋转部分
MatrixXd T_BS_t;  // 平移部分

// IMU积分参数
bool MIDINTEGRATION;          // 是否使用中点积分
bool REINTEGRATIONBYJACOBIAN; // 是否使用Jacobian更新重新积分

/**
 * @brief 从YAML配置文件读取参数
 * @param configdir 配置文件路径
 * 
 * 读取的参数包括：
 * 1. 特征跟踪参数（max_cnt, min_dist, F_threshold等）
 * 2. 初始化参数（numofframestoinit, startframe等）
 * 3. 显示参数（showimg, ifshowpoints等）
 * 4. IMU噪声参数（acc_n, gyr_n, acc_b_n, gyr_b_n）
 * 5. 相机-IMU外参（T_BS）
 * 6. 相机内参（fx, fy, cx, cy, 畸变系数）
 */
void readParameters(string configdir){
    // 打开YAML配置文件
    FileStorage fsettings(configdir,FileStorage::READ);
    if(!fsettings.isOpened()){
        cerr<<"error open config file"<<endl;
    }
    
    // 读取特征跟踪参数
    MAX_CNT = fsettings["max_cnt"];              // 最大特征点数
    MIN_DIST = fsettings["min_dist"];            // 特征点最小间距
    ROW = fsettings["image_height"];             // 图像高度
    COL = fsettings["image_width"];              // 图像宽度
    F_THRESHOLD = fsettings["F_threshold"];      // 基础矩阵RANSAC阈值
    NUMOFFRAMESTOINIT=fsettings["numofframestoinit"];  // 初始化帧数
    EQUALIZE=fsettings["equalize"];              // 是否直方图均衡化
    STARTFRAME=fsettings["startframe"];          // 开始初始化的帧ID
    cout<<STARTFRAME<<endl;
    
    // 读取布尔类型参数（YAML中布尔值需要特殊处理）
    fsettings["showimg"]>>SHOWIMG;               // 是否显示图像
    cout<<SHOWIMG<<endl;
    fsettings["ifshowpoints"]>>IFSHOWPOINTS;     // 是否显示点云
    fsettings["midintegration"]>>MIDINTEGRATION; // 是否中点积分
    fsettings["reintegrationbyjacoba"]>>REINTEGRATIONBYJACOBIAN;  // 是否Jacobian重新积分

    // 读取IMU噪声参数
    ACC_N=fsettings["acc_n"];       // 加速度计噪声
    ACC_B_N=fsettings["acc_b_n"];   // 加速度计偏置噪声
    GYR_N=fsettings["gyr_n"];       // 陀螺仪噪声
    GYR_B_N=fsettings["gyr_b_n"];   // 陀螺仪偏置噪声

    // 读取相机-IMU外参
    Mat T_BS_R_cvmat,T_BS_t_cvmat,T_BS_T_cvmat;
    fsettings["extrinsicRotation"]>>T_BS_R_cvmat;      // 旋转矩阵
    fsettings["extrinsicTranslation"]>>T_BS_t_cvmat;   // 平移向量
    fsettings["T_BS"]>>T_BS_T_cvmat;                   // 完整变换矩阵

    // 转换为Eigen格式
   cv2eigen(T_BS_R_cvmat,T_BS_R);
   cv2eigen(T_BS_t_cvmat,T_BS_t);
   cv2eigen(T_BS_T_cvmat,T_BS);

    // 读取相机内参
    Cameramodel::imagewidth=fsettings["image_width"];
    Cameramodel::imageheight=fsettings["image_height"];
    
    // 读取畸变参数
    FileNode n=fsettings["distortion_parameters"];
    Cameramodel::k1 = static_cast<float>(n["k1"]);
    Cameramodel::k2 = static_cast<float>(n["k2"]);
    Cameramodel::p1 = static_cast<float>(n["p1"]);
    Cameramodel::p2 = static_cast<float>(n["p2"]);

    // 读取投影参数（焦距和主点）
    n = fsettings["projection_parameters"];
    Cameramodel::fx = static_cast<float>(n["fx"]);
    Cameramodel::fy = static_cast<float>(n["fy"]);
    Cameramodel::cx = static_cast<float>(n["cx"]);
    Cameramodel::cy = static_cast<float>(n["cy"]);


}
