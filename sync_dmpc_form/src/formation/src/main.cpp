
#include <iostream>
#include "ros/ros.h"
#include "mymsg/refpos.h"

ros::Publisher refPointPub;

class refPoint {
public:
    double x;
    double y;
    double theta;
public:
    refPoint(double x_, double y_, double theta_) :x(x_), y(y_), theta(theta_) { }
    refPoint() :x(0.0), y(0.0), theta(0.0) { }
};
const int N = 3;//目标点数量，todo:改为argv[]指定
std::vector<refPoint> refPoints;
const double pi = 3.1416;

void Init(ros::NodeHandle n);
void GenerateRefPoint(refPoint);
refPoint GetBasePoint(ros::Duration);

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "formation");
    ros::NodeHandle n;
    ros::Rate loopRate(10);
    Init(n);
    ros::Time beginTime = ros::Time::now();
    while (ros::ok()) {
        ros::Time curTime = ros::Time::now();
        ros::Duration dur = curTime - beginTime;//计算距离开始时的时间差
        GenerateRefPoint(GetBasePoint(dur), dur);
        mymsg::refpos pospub;
        refPointPub.publish(pospub);
        ros::spinOnce();
        loopRate.sleep();
    }
}

void GenerateRefPoint(refPoint base, ros::Duration t) {//todo:实现坐标转换
    enum pattern {
        singleHorizontal,
        singleVertical,
        singleSlope,
        forwardTriangle,
        backwardTriangle,
        circle,
    };
    if (rand() % 1000 > 0) {//1k次更改1次
        return;
    }
    std::cout << "pattern changed:";
    refPoints.clear();
    double gap = 3;
    for (size_t i = 0; i < N; i++) {
        refPoints.push_back(base);
    }
    std::vector<refPoint> tmp(3);
    switch (rand() % 6) {
    case singleHorizontal:
        std::cout << "Single Horizontal" << std::endl;
        for (size_t i = 0; i < N; i++) {
            tmp[i].x = -gap * (N - 1) / 2 + gap * i;
        }
        break;
    case singleVertical:
        std::cout << "Single Vertical" << std::endl;
        for (size_t i = 0; i < N; i++) {
            tmp[i].y = gap * (N - 1) / 2 - gap * i;
        }
        break;
    case singleSlope:
        std::cout << "Single Slope" << std::endl;
        for (size_t i = 0; i < N; i++) {
            tmp[i].x = -gap * (N - 1) / 2 + gap * i;
            tmp[i].y = gap * (N - 1) / 2 - gap * i;
        }
        break;
    case forwardTriangle:
        std::cout << "Forward Triangle" << std::endl;
        for (size_t i = 0; i < N; i++) {
            tmp[i].x = -gap * (N - 1) / 2 + gap * i;
            if (i <= N / 2) {
                tmp[i].y = -gap * (N - 1) / 2 + 2* gap * i;
            }
            else {
                tmp[i].y = gap * (N - 1) / 2 - 2 * gap * i;
            }
        }
        break;
    case backwardTriangle:
        std::cout << "Backward Triangle" << std::endl;
        for (size_t i = 0; i < N; i++) {
            tmp[i].x = -gap * (N - 1) / 2 + gap * i;
            if (i >= N / 2) {
                tmp[i].y = -gap * (N - 1) / 2 + 2 * gap * i;
            }
            else {
                tmp[i].y = gap * (N - 1) / 2 - 2 * gap * i;
            }
        }
        break;
    case circle:
        std::cout << "Circle" << std::endl;
        const double R = 5, phi = 2 * pi / N;
        for (size_t i = 0; i < N; i++) {
            tmp[i].x = R * cos(i * phi);
            tmp[i].y = R * sin(i * phi);
        }

        break;
    };
}

refPoint GetBasePoint(ros::Duration t) {
    // double omega = 0.1;
    // double R = 5;
    // refPoint base;
    // base.x = R * cos(omega * t.toSec());
    // base.y = R * sin(omega * t.toSec());
    // base.theta = omega * t.toSec() + pi / 2;
    
    refPoint base;
    double vx = 0, vy = 2;
    base.x = vx * t.toSec();
    base.y = vy * t.toSec();
    base.theta = 0;
    return base;
}