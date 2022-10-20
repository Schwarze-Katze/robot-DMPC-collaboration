
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
void GenerateRefPoint(refPoint, double);
refPoint GetBasePoint(double);

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "formation");
    ros::NodeHandle n;
    ros::Rate loopRate(5);
    Init(n);
    auto beginTime = ros::Time::now().toSec();
    while (ros::ok()) {
        auto curTime = ros::Time::now().toSec();
        auto dur = curTime - beginTime;//计算距离开始时的时间差
        GenerateRefPoint(GetBasePoint(dur), dur);
        mymsg::refpos pospub;
        for (auto tmp : refPoints) {
            pospub.xr.push_back(tmp.x);
            pospub.yr.push_back(tmp.y);
            pospub.thetar.push_back(tmp.theta);
        }
        refPointPub.publish(pospub);
        ros::spinOnce();
        loopRate.sleep();
    }
}

void Init(ros::NodeHandle n) {
    refPointPub = n.advertise<mymsg::refpos>("formation", 10);
}

void GenerateRefPoint(refPoint base, double t) {//todo:实现坐标转换
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
    refPoints.clear();//清空参考点，不会影响已经pub的消息
    double gap = 3;
    for (size_t i = 0; i < N; i++) {
        refPoints.push_back(base);
    }
    switch (rand() % 6) {
    case singleHorizontal:
        std::cout << "Single Horizontal" << std::endl;
        for (size_t i = 0; i < N; i++) {
            refPoints[i].x = -gap * (N - 1) / 2 + gap * i;
        }
        break;
    case singleVertical:
        std::cout << "Single Vertical" << std::endl;
        for (size_t i = 0; i < N; i++) {
            refPoints[i].y = gap * (N - 1) / 2 - gap * i;
        }
        break;
    case singleSlope:
        std::cout << "Single Slope" << std::endl;
        for (size_t i = 0; i < N; i++) {
            refPoints[i].x = -gap * (N - 1) / 2 + gap * i;
            refPoints[i].y = gap * (N - 1) / 2 - gap * i;
        }
        break;
    case forwardTriangle:
        std::cout << "Forward Triangle" << std::endl;
        for (size_t i = 0; i < N; i++) {
            refPoints[i].x = -gap * (N - 1) / 2 + gap * i;
            if (i <= N / 2) {
                refPoints[i].y = -gap * (N - 1) / 2 + 2 * gap * i;
            }
            else {
                refPoints[i].y = gap * (N - 1) / 2 - 2 * gap * i;
            }
        }
        break;
    case backwardTriangle:
        std::cout << "Backward Triangle" << std::endl;
        for (size_t i = 0; i < N; i++) {
            refPoints[i].x = -gap * (N - 1) / 2 + gap * i;
            if (i >= N / 2) {
                refPoints[i].y = -gap * (N - 1) / 2 + 2 * gap * i;
            }
            else {
                refPoints[i].y = gap * (N - 1) / 2 - 2 * gap * i;
            }
        }
        break;
    case circle:
        std::cout << "Circle" << std::endl;
        const double R = 5, phi = 2 * pi / N;
        for (size_t i = 0; i < N; i++) {
            refPoints[i].x = R * cos(i * phi);
            refPoints[i].y = R * sin(i * phi);
        }
        break;
    };
}

refPoint GetBasePoint(double t) {
    // double omega = 0.1;
    // double R = 5;
    // refPoint base;
    // base.x = R * cos(omega * t.toSec());
    // base.y = R * sin(omega * t.toSec());
    // base.theta = omega * t.toSec() + pi / 2;
    
    refPoint base;
    double vx = 0, vy = 2;
    base.x = vx * t;
    base.y = vy * t;
    base.theta = 0;
    return base;
}