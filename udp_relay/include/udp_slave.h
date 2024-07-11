#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

class UDPSlave {
public:
    UDPSlave();

    ~UDPSlave();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void catCmdCallback(const std_msgs::Bool::ConstPtr& msg);

    void transferData();

private:
    int recv_sock, send_sock;
    struct sockaddr_in local_addr, remote_addr;
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subs;
    std::vector<ros::Publisher> pubs;
    nav_msgs::Odometry odom_buf;
    std_msgs::Bool bool_buf;
    std::mutex _mtx;
    
};