
#ifndef _VEHICLE_H
#define _VEHICLE_H
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
class Vehicle {
private:
    double d_;
    double Ts_;
    double x_;
    double y_;
    double theta_;
    int idx_;
    ros::Subscriber odomSub;
    ros::Publisher commandPub;

public:
    // Vehicle() :x_(0.0), y_(0.0), theta_(0.0), Ts_(0.1), d_(1) { };
    Vehicle() = delete;
    Vehicle(ros::NodeHandle n, int idx, double init_x, double init_y, double init_theta, double Ts, double d) : x_(init_x), y_(init_y), theta_(init_theta), Ts_(Ts), d_(d), idx_(idx) {
        odomSub = n.subscribe<nav_msgs::Odometry>("/robot" + std::to_string(idx) + "/Odometry", 10, boost::bind(&Vehicle::OdomSubCallback, this, _1));
        commandPub = n.advertise<geometry_msgs::Twist>("/robot" + std::to_string(idx) + "/cmd_vel", 10);
    };
    ~Vehicle() { };
    double get_x() { return x_; }
    double get_y() { return y_; }
    double get_theta() { return theta_; }
    void OdomSubCallback(const nav_msgs::Odometry::ConstPtr& pOdom);
    void UpdateStates(const double& xr, const double& yr, const double& thetar, const double& v, const double& angle);
    void SendCommand(const double& xr, const double& yr, const double& thetar, const double& v, const double& angle);
};
void Vehicle::UpdateStates(const double& xr, const double& yr, const double& thetar, const double& v, const double& angle) {
    // x_ += Ts_ * v * std::cos(theta_);
    // y_ += Ts_ * v * std::sin(theta_);
    // theta_ += Ts_ * angle;
    SendCommand(xr, yr, thetar, v, angle);
    return;
}
void Vehicle::OdomSubCallback(const nav_msgs::Odometry::ConstPtr& pOdom) {
    // return;
    x_ = pOdom->pose.pose.position.x;
    y_ = pOdom->pose.pose.position.y;
    auto &q = pOdom->pose.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    theta_ = std::atan2(siny_cosp, cosy_cosp);
}
void Vehicle::SendCommand(const double& xr, const double& yr, const double& thetar, const double& v, const double& angle) {
    geometry_msgs::Twist twist;
    twist.angular.z = angle;
    twist.linear.x = v;
    if (idx_ == 1)
    {
        twist.linear.x = std::min(twist.linear.x, 0.4);
    }
    
    twist.linear.z = thetar;
    commandPub.publish(twist);
}
#endif
