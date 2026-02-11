// featurepoints.cpp - 特征点管理实现
// FeaturePoints类封装了单个特征点的所有信息：
// - 唯一ID
// - 在所有帧中的观测（像素坐标、归一化坐标）
// - 三角化后的3D位置
// - 三角化状态

#include "featurepoints.h"

// 静态成员变量：全局特征点ID计数器
int FeaturePoints::id=0;

// 默认构造函数
FeaturePoints::FeaturePoints(){

}

void FeaturePoints::featurManage()
{

}

/**
 * @brief 构造函数 - 创建新的特征点
 * @param p 特征点像素坐标
 * @param firstsawframe 首次观测到该特征点的帧ID
 * 
 * 初始化流程：
 * 1. 分配唯一ID
 * 2. 记录首次观测帧
 * 3. 像素坐标去畸变并转换为归一化坐标
 * 4. 初始化三角化状态为false
 */
FeaturePoints::FeaturePoints(Point2f p,int firstsawframe){
    pointid=id;  // 分配当前ID
    id++;        // ID计数器递增

    // 记录首次观测
    inWithcframes.push_back(firstsawframe);
    raw_pixelpoints.push_back(make_pair(firstsawframe,p));
    
    // 坐标转换流程：
    // 1. 像素坐标 -> 归一化平面坐标（带畸变）
    // 2. 去畸变
    // 3. 归一化坐标 -> 像素坐标（无畸变）
    Point3f pnorm,un_pnorm;
    Point2f un_p;
    Cameramodel::pixelToNorm(p,pnorm);        // 像素到归一化（带畸变）
    Cameramodel::undistoredPoints(pnorm,un_pnorm);  // 去畸变
    normpoints.push_back(make_pair(firstsawframe,un_pnorm));  // 存储3D归一化坐标
    Cameramodel::normToPixel(un_pnorm,un_p);  // 归一化到像素（无畸变）
    pixelpoints.push_back(make_pair(firstsawframe ,un_p));  // 存储去畸变后的像素坐标
    
    // 存储2D归一化坐标（用于三角化）
    Point2f temp;
    temp.x=un_pnorm.x;
    temp.y=un_pnorm.y;
    normpoints2d.push_back(make_pair(firstsawframe,temp));
    
    // 初始化三角化状态
    triangulated=false;
}

/**
 * @brief 添加新的观测
 * @param idoftheframe 观测到该特征点的帧ID
 * @param p 去畸变后的像素坐标
 * @param raw_p 原始像素坐标（带畸变）
 * 
 * 当特征点在新的帧中被跟踪到时，记录该观测
 */
void FeaturePoints::addFrameThatsaw(int idoftheframe,Point2f &p,Point2f &raw_p)
{
    // 记录观测帧ID
    inWithcframes.push_back(idoftheframe);
    
    // 存储原始像素坐标
    raw_pixelpoints.push_back(make_pair(idoftheframe,raw_p));
    
    // 存储去畸变后的像素坐标
    pixelpoints.push_back(make_pair(idoftheframe,p));
    
    // 转换为归一化坐标
    Point3f pnorm;
    Cameramodel::pixelToNorm(p,pnorm);
    normpoints.push_back(make_pair(idoftheframe,pnorm));
    
    // 存储2D归一化坐标（去掉Z分量）
    Point2f temp;
    temp.x=pnorm.x;
    temp.y=pnorm.y;
    normpoints2d.push_back(make_pair(idoftheframe,temp));
}

