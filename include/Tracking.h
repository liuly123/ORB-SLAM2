/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public://声明函数
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // 预处理输入和调用Track()。提取特征并执行立体匹配
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // 加载新设置
    // 焦距应相似，否则投影点时比例预测将失败。
    // TODO: 修改mappoint::predictscale以考虑焦距
    void ChangeCalibration(const string &strSettingPath);//根据配置文件载入参数

    // 如果您已经停用了局部制图，并且只想定位相机，请使用此函数。
    void InformOnlyTracking(const bool &flag);


public://声明变量

    // 跟踪状态
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // 输入传感器
    int mSensor;

    // 当前帧
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // 初始化变量（单目，单目才需要初始化）
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // 列表（用于在执行结束时恢复完整的摄像机轨迹）。 我们存储每个帧的参考关键帧及其相对变换
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // 如果取消激活本地制图并且我们仅执行定位，则为True
    bool mbOnlyTracking;

    void Reset();

protected:

    // 主要跟踪函数，它独立于输入传感器
    void Track();

    // 双目和RGB-D的地图初始化
    void StereoInitialization();

    // 单目的地图初始化
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // 如果仅执行定位，则当与地图中的点没有匹配时，此标志为true。
    // 如果有匹配的时间足够长，仍跟踪将继续。 在这种情况下，我们正在进行视觉里程计。
    // 系统将尝试进行重定位以恢复到地图的“零漂移”定位。
    bool mbVO;

    //  其他线程的指针
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    // ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    // BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // 初始化实例（仅针对单目）
    Initializer* mpInitializer;

    //局部地图
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    // Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // 地图
    Map* mpMap;

    // 相机参数
    cv::Mat mK;//内参矩阵K
    cv::Mat mDistCoef;//二次畸变参数矩阵
    float mbf;//红外投影仪基线时间fx（大约）

    //新关键帧规则（根据fps）
    int mMinFrames;
    int mMaxFrames;

    // 阈值近/远点
    // 双目 RGBD传感器附近的点被认为是可靠的并且仅从一帧插入。 远点在两个关键帧中需要匹配。
    float mThDepth;

    // 仅用于RGB-D输入。对于某些数据集（如tum），将缩放depthmap值。
    float mDepthMapFactor;//DepthScale

    //帧中的当前匹配项
    int mnMatchesInliers;

    //最新一帧、关键帧和重新定位信息
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //运动模型
    cv::Mat mVelocity;

    //颜色顺序（True为RGB、False为BGR、灰度忽略）
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
