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
#include <mutex>
#include <dynamic_reconfigure/server.h>
#include "udp_relay/initialPoseConfig.h"

class UDPMaster {
public:
    UDPMaster();

    UDPMaster(int _i);

    ~UDPMaster();

    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void catCmdCallback(const std_msgs::Bool::ConstPtr& msg);

    void transferData();

    int self_id, remote_id;
private:
    int recv_sock, send_sock;
    struct sockaddr_in local_addr, remote_addr;
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subs;
    std::vector<ros::Publisher> pubs;
    geometry_msgs::Twist twist_buf;
    std_msgs::Bool bool_buf;
    std::mutex _mtx;
};

void reconfigureCallback(udp_relay::initialPoseConfig& config, uint32_t level);