#include "udp_master.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_relay");
    ros::NodeHandle nh("vehicle1");
    UDPMaster udpComm(3);
    ros::Rate loop_rate(50);

    ros::Publisher master_twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher master_odom_pub = nh.advertise<nav_msgs::Odometry>("Odometry", 1000);

    auto twistCallback = [&](const geometry_msgs::Twist::ConstPtr& msg) {
        ROS_INFO("relay master twist");
        master_twist_pub.publish(*msg);
        };
    auto odomCallback = [&](const nav_msgs::Odometry::ConstPtr& msg) {
        ROS_INFO("relay master odom");
        master_odom_pub.publish(*msg);
        };

    ros::Subscriber master_twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, twistCallback);
    ros::Subscriber master_odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1000, odomCallback);

    while (ros::ok()) {
        udpComm.receiveData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
