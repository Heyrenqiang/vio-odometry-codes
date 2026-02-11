// feature_track.cpp - 特征点跟踪实现
// 实现基于Lucas-Kanade光流法的特征点跟踪
// 包括特征检测、跟踪、外点剔除等功能

#include "feature_track.h"

// 构造函数 - 带图像列表
Feature_Track::Feature_Track(const vector<string> strimg,const vector<double> timestamps)
    :numofimgs(int(strimg.size()))
{

}

// 默认构造函数
Feature_Track::Feature_Track()
{

}

/**
 * @brief 检查点是否在图像边界内
 * @param pt 待检查的点
 * @return 是否在边界内
 * 
 * 边界大小为BORDER_SIZE=1，避免特征点在图像边缘
 */
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

/**
 * @brief 根据状态向量缩减点集（用于去除外点）
 * @param v 点集
 * @param status 状态向量（1表示保留，0表示剔除）
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    size_t j = 0;
    for (size_t i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

/**
 * @brief 根据状态向量缩减整数向量
 * @param v 整数向量（特征点ID）
 * @param status 状态向量
 */
void reduceVector(vector<int> &v, vector<uchar> status)
{
    size_t j = 0;
    for (size_t i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

/**
 * @brief 特征点跟踪 - 旧版本（直接操作Frame对象）
 * @param img 当前图像
 * @param frames 所有帧的集合
 * @param frame 当前帧对象
 * @param featurePoints 所有特征点的集合
 * 
 * 跟踪流程：
 * 1. 如果是第一帧，检测新的特征点
 * 2. 否则使用LK光流跟踪上一帧的特征点
 * 3. 去除图像边界外的点
 * 4. 使用F矩阵进行外点剔除
 * 5. 补充新的特征点以保持数量
 */
void Feature_Track::trackFeaturePoints(const Mat& img,vector<Frame> &frames,
                                       Frame &frame,vector<FeaturePoints> &featurePoints){
    // 第一帧：检测新的特征点
    if(frame.id==0){
        previmg=img;
        curimg=img;
        setMask();  // 设置掩膜避免特征点过于密集
        // 使用Shi-Tomasi角点检测（goodFeaturesToTrack）
        goodFeaturesToTrack(curimg,point2fs,MAX_CNT,0.01,MIN_DIST,mask,3,3,0,0.04);
        for(auto &p:point2fs){
            curpoint2fs.push_back(p);
            FeaturePoints featurepoint(p,frame.id);
            frame.addFeaturePoints(featurepoint);
            featurePoints.push_back(featurepoint);
        }
    }
    // 后续帧：光流跟踪
    else{
        curimg=img;
        vector<uchar> status;  // 光流跟踪状态
        vector<float> err;     // 跟踪误差
        
        // LK光流跟踪：从previmg到curimg跟踪prevpoint2fs
        calcOpticalFlowPyrLK(previmg,curimg,prevpoint2fs,curpoint2fs,status,err,cv::Size(21, 21),3);

        vector<int> indexs;
        for(int k=0;k<MAX_CNT;k++){
            indexs.push_back(k);
        }
        
        // 去除图像边界外的点
        for (size_t i = 0; i < curpoint2fs.size(); i++){
            if (status[i] && !inBorder(curpoint2fs[i]))
                status[i] = 0;
        }
        reduceVector(prevpoint2fs, status);
        reduceVector(curpoint2fs, status);
        reduceVector(indexs,status);
        
        // 去畸变 - 将像素坐标转换为归一化坐标
        vector<Point2f> un_prevpoint2fs,un_curpoint2fs;
        for(size_t i=0;i<prevpoint2fs.size();i++){
            Point2f temp;
            Cameramodel::removeDistortion(prevpoint2fs[i],temp);
            un_prevpoint2fs.push_back(temp);
            Cameramodel::removeDistortion(curpoint2fs[i],temp);
            un_curpoint2fs.push_back(temp);
        }
        
        // 使用基础矩阵F进行外点剔除（RANSAC）
        rejectWithF(indexs,un_prevpoint2fs,un_curpoint2fs);

        // 如果特征点数量不足，补充新的特征点
        if(MAX_CNT-int(curpoint2fs.size())>0){
            setMask();
            goodFeaturesToTrack(curimg,point2fs,MAX_CNT-int(curpoint2fs.size()),0.01,MIN_DIST,mask,3,3,0,0.04);
            
            // 更新跟踪到的特征点
            for(size_t i=0;i<indexs.size();i++){
                unsigned long trackedid=ulong(frames[frames.size()-2].featurePointscoords_norm[ulong(indexs[i])].first);
                featurePoints[trackedid].addFrameThatsaw(frame.id,un_curpoint2fs[i],curpoint2fs[i]);
                frame.addFeaturePoints(featurePoints[trackedid]);
                frames[frames.size()-2].addTrackedFeaturePointstonext(featurePoints[trackedid].id,un_prevpoint2fs[i]);
                frame.addTrackedFeaturePointstoprevious(featurePoints[trackedid].id,un_curpoint2fs[i]);
            }
            // 创建新的特征点
            for(auto &p:point2fs){
                curpoint2fs.push_back(p);
                FeaturePoints featurepoint(p,frame.id);
                frame.addFeaturePoints(featurepoint);
                featurePoints.push_back(featurepoint);
            }
        }else{
            // 特征点数量足够，只更新跟踪到的点
            for(size_t i=0;i<indexs.size();i++){
                unsigned long trackedid=ulong(frames[frames.size()-2].featurePointscoords_norm[ulong(indexs[i])].first);
                featurePoints[trackedid].addFrameThatsaw(frame.id,un_curpoint2fs[i],curpoint2fs[i]);
                frame.addFeaturePoints(featurePoints[trackedid]);
                frames[frames.size()-2].addTrackedFeaturePointstonext(featurePoints[trackedid].id,un_prevpoint2fs[i]);
                frame.addTrackedFeaturePointstoprevious(featurePoints[trackedid].id,un_curpoint2fs[i]);
            }
        }

        // 可视化：显示特征点跟踪结果
        if(SHOWIMG){
            Mat imgshowlast=previmg.clone();
            for(auto kp:frames[frames.size()-2].trackedFeaturePointstonext){
                circle(imgshowlast,kp.second,4,Scalar(0,240,1),3);
            }
            imshow("cornerslast",imgshowlast);
            Mat imgshow=curimg.clone();
            for(auto kp: frame.trackedFeaturePointstoprevious){
                circle(imgshow,kp.second,4,Scalar(0,240,0),3);
            }
            imshow("corners",imgshow);
            waitKey(0);
        }
    }
    // 更新上一帧图像和特征点
    previmg=curimg;
    prevpoint2fs=curpoint2fs;
}


/**
 * @brief 特征点跟踪 - 新版本（返回跟踪结果，不直接操作Frame）
 * @param img 当前图像
 * @param frameid 当前帧ID
 * @param featuresinprevframe 前一帧中的特征点ID列表
 * @param newaddedpoints 输出：新检测的特征点
 * @param trackedpointsincomingframes 输出：跟踪到的特征点及其坐标
 * 
 * 返回值通过参数传递，便于上层灵活处理
 */
void Feature_Track::trackFeaturePoints2(const Mat& img,int frameid,vector<int> featuresinprevframe,vector<Point2f> &newaddedpoints,
                                       vector<pair<int,pair<Point2f,Point2f>>>& trackedpointsincomingframes){

    // 第一帧：检测新的特征点
    if(frameid==0){
        previmg=img;
        curimg=img;
        setMask();
        goodFeaturesToTrack(curimg,point2fs,MAX_CNT,0.01,MIN_DIST,mask,3,3,0,0.04);
        for(auto &p:point2fs){
            curpoint2fs.push_back(p);
            newaddedpoints.push_back(p);
        }
    }
    // 后续帧：光流跟踪
    else{
        curimg=img;
        vector<uchar> status;
        vector<float> err;
        
        // LK光流跟踪
        calcOpticalFlowPyrLK(previmg,curimg,prevpoint2fs,curpoint2fs,status,err,cv::Size(21, 21),3);

        vector<int> indexs;
        for(int k=0;k<MAX_CNT;k++){
            indexs.push_back(k);
        }
        
        // 去除边界外的点
        for (size_t i = 0; i < curpoint2fs.size(); i++){
            if (status[i] && !inBorder(curpoint2fs[i]))
                status[i] = 0;
        }
        reduceVector(prevpoint2fs, status);
        reduceVector(curpoint2fs, status);
        reduceVector(indexs,status);
        
        // 去畸变
        vector<Point2f> un_prevpoint2fs,un_curpoint2fs;
        for(size_t i=0;i<prevpoint2fs.size();i++){
            Point2f temp;
            Cameramodel::removeDistortion(prevpoint2fs[i],temp);
            un_prevpoint2fs.push_back(temp);
            Cameramodel::removeDistortion(curpoint2fs[i],temp);
            un_curpoint2fs.push_back(temp);
        }
        
        // F矩阵外点剔除
        rejectWithF(indexs,un_prevpoint2fs,un_curpoint2fs);
        
        // 收集跟踪到的特征点信息
        for(size_t i=0;i<indexs.size();i++){
            unsigned long trackedid=ulong(featuresinprevframe[ulong(indexs[i])]);
            trackedpointsincomingframes.push_back(make_pair(trackedid,make_pair(un_curpoint2fs[i],curpoint2fs[i])));
        }
        
        // 补充新的特征点
        if(MAX_CNT-int(curpoint2fs.size())>0){
            setMask();
            goodFeaturesToTrack(curimg,point2fs,MAX_CNT-int(curpoint2fs.size()),0.01,MIN_DIST,mask,3,3,0,0.04);
            for(auto &p:point2fs){
                curpoint2fs.push_back(p);
                newaddedpoints.push_back(p);

            }
        }
    }
    // 更新状态
    previmg=curimg;
    prevpoint2fs=curpoint2fs;
}



/**
 * @brief 使用基础矩阵F进行外点剔除
 * @param indexs 特征点索引
 * @param un_curp 当前帧归一化坐标
 * @param un_forwp 下一帧归一化坐标
 * 
 * 使用RANSAC算法估计基础矩阵，剔除不符合几何约束的特征点
 */
void Feature_Track::rejectWithF(vector<int> &indexs,vector<Point2f> &un_curp,vector<Point2f> &un_forwp)
{
    vector<uchar> status;
    // RANSAC估计基础矩阵，阈值F_THRESHOLD，置信度0.99
    findFundamentalMat(un_curp, un_forwp, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
    reduceVector(indexs,status);
    reduceVector(un_curp,status);
    reduceVector(un_forwp,status);
    reduceVector(prevpoint2fs,status);
    reduceVector(curpoint2fs,status);
}

/**
 * @brief 设置特征点检测掩膜
 * 
 * 在已有特征点周围创建圆形掩膜，避免新检测的特征点过于密集
 * 圆半径为MIN_DIST，确保特征点间最小距离
 */
void Feature_Track::setMask(){
    mask = Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    for(auto &p : curpoint2fs){
        int min=int(MIN_DIST);
        circle(mask, p, min, 0, -1);  // 在特征点周围画黑色圆（禁止检测区域）
    }
}





































