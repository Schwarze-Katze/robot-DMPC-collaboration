#include "udp_master.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_relay");
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<udp_relay::initialPoseConfig> server;
    dynamic_reconfigure::Server<udp_relay::initialPoseConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);

    std::string cfg_path;
    ros::param::get("/cfg_path", cfg_path);
    ROS_INFO("config path: %s", cfg_path.c_str());
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
        extern std::vector<double> x_diff, y_diff, yaw_diff;
        nav_msgs::Odometry odom_msg = *msg;
        auto id = udpComm[0]->self_id;
        odom_msg.pose.pose.position.x += x_diff[id - 1];
        odom_msg.pose.pose.position.y += y_diff[id - 1];
        ROS_INFO("relay master odom");
        master_odom_pub.publish(odom_msg);
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
