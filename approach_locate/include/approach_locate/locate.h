#ifndef LOCATE_H
#define LOCATE_H
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Pose.h>

//调试窗口标题
#define DEBUGUI_TITLE "ArUco Detection"

// 二维码位姿结构体
typedef struct ArUcoPose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double qx;
    double qy;
    double qz;
    double qw;

    void rpyToQuaternion() {
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
    }
} ArUcoPose_t;

// ArUco定位算法
class ArUcoLocation {
public:
    // 初始化摄像头和参数
    bool init(int webcamIndex, double hViewAngle, bool debugUI);
    // 获取ArUco码的位姿
    bool getArUcoPose(ArUcoPose_t* arucoPose);
    // 销毁资源
    bool destroy();

private:
    // 摄像头
    cv::VideoCapture capture;
    // 摄像头上下视角
    double hViewAngle;
    // 是否开启调试窗口
    bool debugUI;
    // 灰度图
    cv::Mat grayFrame;
    // ArUco字典
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    // ArUco检测参数
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

private:
    // 计算位姿
    bool getArUcoPose(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners, ArUcoPose_t* arucoPose);
};

#endif