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


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // 初始化SLAM系统，启动本地绘图、回环检测、Viewer线程
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    // 双目跟踪方法：图像必须同步和矫正，输入图像为RGB(CV_8UC3)或灰度图(CV_8U)，
    // RGB图像会被转换为灰度图，返回相机的位姿矩阵
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // RGBD跟踪方法：深度图必须与RGB图对应，输入的图像为RGB(CV_8UC3)或灰度图(CV_8U)，
    // 深度图像为Float (CV_32F)，返回相机位姿矩阵（失败则为空）

    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // 单目跟踪方法：输入的图像为RGB(CV_8UC3)或灰度图(CV_8U)，
    // 返回相机位姿矩阵（失败则为空）
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // 停止地图生成，仅执行相机位姿跟踪
    void ActivateLocalizationMode();
    // 恢复地图生成
    void DeactivateLocalizationMode();

    // 如果自上次调用此函数以来发生了较大的地图更改（如进行了循环闭合或全局BA），则返回true
    bool MapChanged();

    // 重置系统（清空地图）
    void Reset();

    //所有线程都将被请求完成，等待所有线程完成。保存轨迹之前必须调用此函数。
    void Shutdown();

    // 以TUM RGB-D数据集格式保存像机轨迹。仅适用于双目和RGB-D。
    // 此方法不适用于单目。要调用此函数，首先要调用Shutdown()
    // 请参阅格式详细信息：http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // 以TUM RGB-D数据集格式保存像机轨迹。适用于所有传感器。
    // 要调用此函数，首先要调用Shutdown()
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // 以KITTI数据集格式保存像机轨迹。仅适用于双目和RGB-D。
    // 此方法不适用于单目。要调用此函数，首先要调用Shutdown()
    // 请参阅以下格式详细信息：http://www.cvlibs.net/datasets/kitti/eval
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // 获取最近一帧处理的信息
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    // ***************************************以下是一些private数据类型***********************************
    // 输入数据的传感器类型
    eSensor mSensor;

    // ORB字典
    ORBVocabulary* mpVocabulary;

    // 关键帧数据库
    KeyFrameDatabase* mpKeyFrameDatabase;

    // 存储指向所有KeyFrame和MapPoints的指针的地图结构
    Map* mpMap;

    // 跟踪器：它接收一帧并计算相应的相机姿势。它还决定何时插入新的KeyFrame，创建一些新的MapPoints，并在跟踪失败时执行重定位
    Tracking* mpTracker;

    // 局部制图器：它管理局部地图并执行局部BA
    LocalMapping* mpLocalMapper;

    // 回环检测处理：它会搜索每个新关键帧的回环。 如果存在回环，那么之后执行位姿图优化和完整BA（在新线程中）
    LoopClosing* mpLoopCloser;

    //MapViewer：绘制地图和相机位姿（使用Pangolin），还有另一个窗口显示当前摄像头的画面
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // 上面定义的处理程序对用的线程：局部制图器、回环检测处理、MapViewer
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // 重置标志
    std::mutex mMutexReset;// 互斥体，用于多线程加锁
    bool mbReset;

    // 更改模式标志（改变的时候才为true，才需要执行操作）
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;//是否激活定位模式（不建图）
    bool mbDeactivateLocalizationMode;//是否取消激活定位模式

    int mTrackingState;// 跟踪状态
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;//用于修改数据时加锁
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
