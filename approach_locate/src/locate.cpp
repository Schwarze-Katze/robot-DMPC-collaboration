#include "approach_locate/locate.h"

bool ArUcoLocation::init(int webcamIndex, double hViewAngle, bool debugUI) {
    // 打开摄像头
    capture.open("udp://127.0.0.1:15000");
    if (!capture.isOpened())
        return false;

    this->hViewAngle = hViewAngle;
    this->debugUI = debugUI;

    // 创建ArUco字典和检测参数
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    detectorParams = cv::aruco::DetectorParameters::create();

    // 如果开启调试模式，则创建窗口
    if (debugUI) {
        cv::namedWindow(DEBUGUI_TITLE, cv::WINDOW_AUTOSIZE);
    }

    return true;
}

bool ArUcoLocation::getArUcoPose(ArUcoPose_t* arucoPose) {
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
        // 显示调试窗口
        if (debugUI) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            cv::imshow(DEBUGUI_TITLE, frame);
            cv::waitKey(10);
        }

        // 计算位姿
        return getArUcoPose(ids, corners, arucoPose);
    }
    else {
        if (debugUI) {
            cv::imshow(DEBUGUI_TITLE, frame);
            cv::waitKey(10);
        }
        return false;
    }
}

bool ArUcoLocation::getArUcoPose(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners, ArUcoPose_t* arucoPose) {
    // 这里的实现假设检测到的第一个ArUco码是需要计算位姿的对象
    // 可以根据需求扩展处理多个ArUco码
    if (corners.empty()) {
        return false;
    }

    // 获取ArUco码的四个顶点
    const std::vector<cv::Point2f>& markerCorners = corners[0];

    // 假设你有预定的ArUco码实际尺寸 qrSize
    double qrSize = 0.05; // 1厘米的ArUco码

    // 初始化相机内参矩阵 (3x3)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1.02226575e+03, 0.00000000e+00, 9.80481177e+02,
        0.00000000e+00, 1.01737850e+03, 4.94844585e+02,
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00);

    // 初始化失真系数 (5x1)
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 4.88910348e-03, -1.82542947e-01, 5.57099089e-04,
        1.95687004e-03, 6.85519085e-01);

    // 计算位姿
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, qrSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    // 提取第一个ArUco码的位姿信息
    cv::Vec3d rotation = rvecs[0];
    cv::Vec3d translation = tvecs[0];

    // 使用旋转和平移向量计算姿态
    arucoPose->x = translation[0];
    arucoPose->y = translation[1];
    arucoPose->z = translation[2];  // z表示距离
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
    arucoPose->roll = yaw;
    arucoPose->pitch = roll;
    arucoPose->yaw = pitch;
    arucoPose->rpyToQuaternion();

    return true;
}

bool ArUcoLocation::destroy() {
    // 销毁窗口
    if (debugUI) {
        cv::destroyWindow(DEBUGUI_TITLE);
    }

    // 释放摄像头
    capture.release();
    return true;
}