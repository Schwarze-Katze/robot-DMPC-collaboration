#include "udp_master.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_relay");
    ros::NodeHandle nh;
    std::string cfg_path;
    ros::param::get("/cfg_path", cfg_path);
    YAML::Node cfg = YAML::LoadFile(cfg_path);
    int remote_cnt = cfg["remote"].size();
    std::vector<std::shared_ptr<UDPMaster>> udpComm;
    for (int i = 0;i < remote_cnt;++i) {
        udpComm.emplace_back(std::make_shared<UDPMaster>(i));
    }
    ros::Rate loop_rate(50);

    ros::Publisher master_twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher master_odom_pub = nh.advertise<nav_msgs::Odometry>("/robot1/Odometry", 1000);

    auto twistCallback = [&](const geometry_msgs::Twist::ConstPtr& msg) {
        ROS_INFO("relay master twist");
        master_twist_pub.publish(*msg);
        };
    auto odomCallback = [&](const nav_msgs::Odometry::ConstPtr& msg) {
        ROS_INFO("relay master odom");
        master_odom_pub.publish(*msg);
        };

    ros::Subscriber master_twist_sub = nh.subscribe<geometry_msgs::Twist>("/robot1/cmd_vel", 1000, twistCallback);
    ros::Subscriber master_odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1000, odomCallback);

    while (ros::ok()) {
        for (auto& target : udpComm) {
            target->transferData();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
