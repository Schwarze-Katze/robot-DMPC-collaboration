#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>

class UDPSlave {
public:
    UDPSlave();

    ~UDPSlave();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void receiveData();

private:
    int recv_sock, send_sock;
    struct sockaddr_in local_addr, remote_addr;
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher twist_pub;
};