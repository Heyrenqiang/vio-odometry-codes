#include "feature_track.h"
Feature_Track::Feature_Track(const vector<string> strimg,const vector<double> timestamps)
    :numofimgs(int(strimg.size()))
{

}

Feature_Track::Feature_Track()
{

}
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    size_t j = 0;
    for (size_t i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    size_t j = 0;
    for (size_t i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
void Feature_Track::trackFeaturePoints(const Mat& img,vector<Frame> &frames,
                                       Frame &frame,vector<FeaturePoints> &featurePoints){
//    if(frame.id==0){
//    curimg=img;
//    setMask();
//    goodFeaturesToTrack(curimg,point2fs,MAX_CNT-int(curpoint2fs.size()),0.01,MIN_DIST,mask,3,3,0,0.04);

//            for(int i=0;i<150;i++){
//                prevpoint2fs.push_back(point2fs[ulong(i)]);

//            }}
//    for(int i=0;i<150;i++){
//        if((rand() / double(RAND_MAX))>0.3){
//            curpoint2fs.push_back(prevpoint2fs[ulong(i)]);
//        }
//    }
//    curimg=img;
//    setMask();
//    if(MAX_CNT-int(curpoint2fs.size())>0){
//        goodFeaturesToTrack(curimg,point2fs,MAX_CNT-int(curpoint2fs.size()),0.01,MIN_DIST,mask,3,3,0,0.04);
//                for(auto &p:point2fs){
//                    prevpoint2fs.push_back(p);
//                    FeaturePoints featurepoint(p,frame.id);
//                    frame.addFeaturePoints(featurepoint);
//                    featurePoints.push_back(featurepoint);
//                }
//    }
//    curpoint2fs.clear();
//    cout<<"dsd"<<endl;
    //if the first frame comes,just find enough features to track
    if(frame.id==0){
        previmg=img;
        curimg=img;
        setMask();
        goodFeaturesToTrack(curimg,point2fs,MAX_CNT,0.01,MIN_DIST,mask,3,3,0,0.04);
        for(auto &p:point2fs){
            curpoint2fs.push_back(p);
            FeaturePoints featurepoint(p,frame.id);
            frame.addFeaturePoints(featurepoint);
            featurePoints.push_back(featurepoint);
        }
    }
    //track the newest frame with the last frame,find matched featurepoints
    else{
        curimg=img;
        vector<uchar> status;
        vector<float> err;
        //LK Optical Flow methods to find matched featurepoints
        calcOpticalFlowPyrLK(previmg,curimg,prevpoint2fs,curpoint2fs,status,err,cv::Size(21, 21),3);

        vector<int> indexs;
        for(int k=0;k<MAX_CNT;k++){
            indexs.push_back(k);
        }
        //remove the featurepoints that outBoder the image
        for (size_t i = 0; i < curpoint2fs.size(); i++){
            if (status[i] && !inBorder(curpoint2fs[i]))
                status[i] = 0;
        }
        reduceVector(prevpoint2fs, status);
        reduceVector(curpoint2fs, status);
        reduceVector(indexs,status);
        //undistort the featurepoints
        vector<Point2f> un_prevpoint2fs,un_curpoint2fs;
        for(size_t i=0;i<prevpoint2fs.size();i++){
            Point2f temp;
            Cameramodel::removeDistortion(prevpoint2fs[i],temp);
            un_prevpoint2fs.push_back(temp);
            Cameramodel::removeDistortion(curpoint2fs[i],temp);
            un_curpoint2fs.push_back(temp);
        }
        //remove featurepoints that did not suit the F Matrix
        rejectWithF(indexs,un_prevpoint2fs,un_curpoint2fs);

        //add new featurepoints to keep enough fetures to track
        if(MAX_CNT-int(curpoint2fs.size())>0){
            setMask();
            goodFeaturesToTrack(curimg,point2fs,MAX_CNT-int(curpoint2fs.size()),0.01,MIN_DIST,mask,3,3,0,0.04);
            //vector<pair<int,pair<Point2f,Point2f>>> watchedpoints;
            for(size_t i=0;i<indexs.size();i++){
                unsigned long trackedid=ulong(frames[frames.size()-2].featurePointscoords_norm[ulong(indexs[i])].first);
                featurePoints[trackedid].addFrameThatsaw(frame.id,un_curpoint2fs[i],curpoint2fs[i]);
                //watchedpoints.push_back(make_pair(trackedid,make_pair(un_curpoint2fs[i],curpoint2fs[i])));
                frame.addFeaturePoints(featurePoints[trackedid]);
                frames[frames.size()-2].addTrackedFeaturePointstonext(featurePoints[trackedid].id,un_prevpoint2fs[i]);
                frame.addTrackedFeaturePointstoprevious(featurePoints[trackedid].id,un_curpoint2fs[i]);
            }
            //creat new featurepoints
            for(auto &p:point2fs){
                curpoint2fs.push_back(p);
                FeaturePoints featurepoint(p,frame.id);
                frame.addFeaturePoints(featurepoint);
                featurePoints.push_back(featurepoint);
            }
        }else{
            //vector<pair<int,pair<Point2f,Point2f>>> watchedpoints;
            for(size_t i=0;i<indexs.size();i++){
                unsigned long trackedid=ulong(frames[frames.size()-2].featurePointscoords_norm[ulong(indexs[i])].first);
                featurePoints[trackedid].addFrameThatsaw(frame.id,un_curpoint2fs[i],curpoint2fs[i]);
                //watchedpoints.push_back(make_pair(trackedid,make_pair(un_curpoint2fs[i],curpoint2fs[i])));
                frame.addFeaturePoints(featurePoints[trackedid]);
                frames[frames.size()-2].addTrackedFeaturePointstonext(featurePoints[trackedid].id,un_prevpoint2fs[i]);
                frame.addTrackedFeaturePointstoprevious(featurePoints[trackedid].id,un_curpoint2fs[i]);
            }
        }

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

        if(false){
            Mat imgshow=curimg.clone();
            vector<int> dds;
            dds.push_back(1);
            dds.push_back(2);
            dds.push_back(5);
            dds.push_back(20);
            dds.push_back(11);
            //dds.push_back(12);
            dds.push_back(15);
            dds.push_back(21);
            dds.push_back(19);
            //dds.push_back(27);
            dds.push_back(30);
            //dds.push_back(32);
            for(size_t k=0;k<dds.size();k++){
                for(size_t i=0;i<featurePoints[1].pixelpoints.size();i++){
                    if(featurePoints[ulong(dds[k])].pixelpoints[i].first==frame.id){
                        circle(imgshow,featurePoints[ulong(dds[k])].raw_pixelpoints[i].second,4,Scalar(0,240,0),3);
                        circle(imgshow,featurePoints[ulong(dds[k])].pixelpoints[i].second,12,Scalar(0,240,0),2);
                        break;
                    }

                }
            }
            imshow("featurs",imgshow);
            waitKey(0);
        }
    }
    previmg=curimg;
    prevpoint2fs=curpoint2fs;
}


void Feature_Track::trackFeaturePoints2(const Mat& img,int frameid,vector<int> featuresinprevframe,vector<Point2f> &newaddedpoints,
                                       vector<pair<int,pair<Point2f,Point2f>>>& trackedpointsincomingframes){

    //if the first frame comes,just find enough features to track
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
    //track the newest frame with the last frame,find matched featurepoints
    else{
        curimg=img;
        vector<uchar> status;
        vector<float> err;
        //LK Optical Flow methods to find matched featurepoints
        calcOpticalFlowPyrLK(previmg,curimg,prevpoint2fs,curpoint2fs,status,err,cv::Size(21, 21),3);

        vector<int> indexs;
        for(int k=0;k<MAX_CNT;k++){
            indexs.push_back(k);
        }
        //remove the featurepoints that outBoder the image
        for (size_t i = 0; i < curpoint2fs.size(); i++){
            if (status[i] && !inBorder(curpoint2fs[i]))
                status[i] = 0;
        }
        reduceVector(prevpoint2fs, status);
        reduceVector(curpoint2fs, status);
        reduceVector(indexs,status);
        //undistort the featurepoints
        vector<Point2f> un_prevpoint2fs,un_curpoint2fs;
        for(size_t i=0;i<prevpoint2fs.size();i++){
            Point2f temp;
            Cameramodel::removeDistortion(prevpoint2fs[i],temp);
            un_prevpoint2fs.push_back(temp);
            Cameramodel::removeDistortion(curpoint2fs[i],temp);
            un_curpoint2fs.push_back(temp);
        }
        //remove featurepoints that did not suit the F Matrix
        rejectWithF(indexs,un_prevpoint2fs,un_curpoint2fs);
        //vector<pair<int,pair<Point2f,Point2f>>> watchedpoints;
        for(size_t i=0;i<indexs.size();i++){
            unsigned long trackedid=ulong(featuresinprevframe[ulong(indexs[i])]);
            trackedpointsincomingframes.push_back(make_pair(trackedid,make_pair(un_curpoint2fs[i],curpoint2fs[i])));
//            featurePoints[trackedid].addFrameThatsaw(frame.id,un_curpoint2fs[i],curpoint2fs[i]);
//            //watchedpoints.push_back(make_pair(trackedid,make_pair(un_curpoint2fs[i],curpoint2fs[i])));
//            frame.addFeaturePoints(featurePoints[trackedid]);
//            frames[frames.size()-2].addTrackedFeaturePointstonext(featurePoints[trackedid].id,un_prevpoint2fs[i]);
//            frame.addTrackedFeaturePointstoprevious(featurePoints[trackedid].id,un_curpoint2fs[i]);
        }
        //add new featurepoints to keep enough fetures to track
        if(MAX_CNT-int(curpoint2fs.size())>0){
            setMask();
            goodFeaturesToTrack(curimg,point2fs,MAX_CNT-int(curpoint2fs.size()),0.01,MIN_DIST,mask,3,3,0,0.04);
            //creat new featurepoints
            for(auto &p:point2fs){
                curpoint2fs.push_back(p);
                newaddedpoints.push_back(p);

            }
        }
    }
    previmg=curimg;
    prevpoint2fs=curpoint2fs;
}



void Feature_Track::rejectWithF(vector<int> &indexs,vector<Point2f> &un_curp,vector<Point2f> &un_forwp)
{
    vector<uchar> status;
    findFundamentalMat(un_curp, un_forwp, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
    reduceVector(indexs,status);
    reduceVector(un_curp,status);
    reduceVector(un_forwp,status);
    reduceVector(prevpoint2fs,status);
    reduceVector(curpoint2fs,status);
}
void Feature_Track::setMask(){
    mask = Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    for(auto &p : curpoint2fs){
        int min=int(MIN_DIST);
        circle(mask, p, min, 0, -1);
    }
}



































