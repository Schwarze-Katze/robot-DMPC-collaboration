#include "approach_locate/locate.h"

bool ArUcoLocation::init(int webcamIndex, bool showVideo) {
    // 打开摄像头
    capture.open(webcamIndex);
    if (!capture.isOpened())
        return false;

    this->showVideo = showVideo;

    // 创建ArUco字典和检测参数
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    detectorParams = cv::aruco::DetectorParameters::create();

    // 初始化相机内参矩阵 (3x3)
    cameraMatrix = (cv::Mat_<double>(3, 3) << 994.94779608, 0.00000000e+00, 949.41958243,
        0.00000000e+00, 994.83295499, 455.9985767,
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00);

    // 初始化失真系数 (5x1)
    distCoeffs = (cv::Mat_<double>(5, 1) << -0.02966125, 0.06407681, -0.00890747,
        -0.0074752, -0.04464296);

    // 如果开启调试模式，则创建窗口
    if (showVideo) {
        cv::namedWindow(DEBUGUI_TITLE, cv::WINDOW_KEEPRATIO);
    }

    return true;
}

bool ArUcoLocation::getArUcoPose(std::vector<ArUcoPose_t>& posevec) {
    // 抓取摄像头帧
    cv::Mat frame;
    capture >> frame;
    if (frame.empty()) {
        return false;
    }
    // 将图像转为灰度图
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

    // 检测ArUco码
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(grayFrame, dictionary, corners, ids, detectorParams);

    // 如果检测到ArUco码
    if (!ids.empty()) {
        bool detect_status = false;
        // 显示调试窗口
        if (showVideo) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            detect_status = getArUcoPose(ids, corners, posevec, frame);

            cv::imshow(DEBUGUI_TITLE, frame);
            cv::waitKey(10);
        }

        // 计算位姿
        return detect_status;
    }
    else {
        if (showVideo) {
            cv::imshow(DEBUGUI_TITLE, frame);
            cv::waitKey(10);
        }
        return false;
    }
}

bool ArUcoLocation::getArUcoPose(std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners, std::vector<ArUcoPose_t>& posevec, cv::Mat& frame) {
    if (ids.empty()) {
        return false;
    }
    for (int i = 0;i < ids.size();++i) {
        ArUcoPose_t arucoPose;
        arucoPose.id = ids[i];

        // 获取ArUco码的四个顶点
        const std::vector<cv::Point2f>& markerCorners = corners[i];

        // 假设你有预定的ArUco码实际尺寸 qrSize
        double qrSize = 0.05; // 1厘米的ArUco码

        // 计算位姿
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, qrSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        // 绘制坐标系
        double axisLength = 0.02;
        cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs, tvecs, axisLength);

        // 提取第一个ArUco码的位姿信息
        cv::Vec3d rotation = rvecs[0];
        cv::Vec3d translation = tvecs[0];

        // 使用旋转和平移向量计算姿态
        arucoPose.x = translation[0];
        arucoPose.y = translation[1];
        arucoPose.z = translation[2];  // z表示距离
        // 计算旋转矩阵并提取roll, pitch, yaw
        cv::Mat rotMat;
        cv::Rodrigues(rotation, rotMat);  // 将旋转向量转换为旋转矩阵

        // 提取欧拉角 (roll, pitch, yaw)
        double sy = sqrt(rotMat.at<double>(0, 0) * rotMat.at<double>(0, 0) + rotMat.at<double>(1, 0) * rotMat.at<double>(1, 0));

        bool singular = sy < 1e-6;  // 判断是否接近平行

        double roll, pitch, yaw;
        if (!singular) {
            roll = atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
            pitch = atan2(-rotMat.at<double>(2, 0), sy);
            yaw = atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
        }
        else {
            roll = atan2(-rotMat.at<double>(1, 2), rotMat.at<double>(1, 1));
            pitch = atan2(-rotMat.at<double>(2, 0), sy);
            yaw = 0;
        }
        roll += M_PI;
        if (roll > M_PI) {
            roll -= 2 * M_PI;
        }

        // 保存roll, pitch, yaw信息
        arucoPose.roll = yaw;
        arucoPose.pitch = roll;
        arucoPose.yaw = pitch;
        arucoPose.rpyToQuaternion();
        posevec.push_back(arucoPose);
    }
    return true;
}

bool ArUcoLocation::destroy() {
    // 销毁窗口
    if (showVideo) {
        cv::destroyWindow(DEBUGUI_TITLE);
    }

    // 释放摄像头
    capture.release();
    return true;
}