#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>

class UDPMaster {
public:
    UDPMaster() = delete;

    UDPMaster(int idx);

    ~UDPMaster();

    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);

    void receiveData();

private:
    int idx;
    int recv_sock, send_sock;
    struct sockaddr_in local_addr, remote_addr;
    ros::NodeHandle nh;
    ros::Subscriber twist_sub;
    ros::Publisher odom_pub;
};