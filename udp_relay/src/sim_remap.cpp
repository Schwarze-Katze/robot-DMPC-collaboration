#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

constexpr size_t m = 3;
std::vector<ros::Publisher> twist_pub, odom_pub;
std::vector<ros::Subscriber> twist_sub, odom_sub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "sim_remap");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    // std::vector<std::function<void(const geometry_msgs::Twist::ConstPtr&)>> twistCallbacks;
    // std::vector<std::function<void(const nav_msgs::Odometry::ConstPtr&)>> odomCallbacks;

    for (int i = 1;i <= m;++i) {
        twist_pub.push_back(nh.advertise<geometry_msgs::Twist>("/robot" + std::to_string(i) + "/drive_controller/cmd_vel", 1000));
        odom_pub.push_back(nh.advertise<nav_msgs::Odometry>("/robot" + std::to_string(i) + "/Odometry", 1000));
        auto twistCallback = [i](const geometry_msgs::Twist::ConstPtr& msg) {
            ROS_INFO("relay twist %d", i);
            twist_pub[i - 1].publish(*msg);
            };
        // twistCallbacks.push_back(twistCallback);
        auto odomCallback = [i](const nav_msgs::Odometry::ConstPtr& msg) {
            ROS_INFO("relay odom %d", i);
            odom_pub[i - 1].publish(*msg);
            };
        // odomCallbacks.push_back(odomCallback);
        twist_sub.push_back(nh.subscribe<geometry_msgs::Twist>("/robot" + std::to_string(i) + "/cmd_vel", 1000, twistCallback));
        odom_sub.push_back(nh.subscribe<nav_msgs::Odometry>("/robot" + std::to_string(i) + "/drive_controller/odom", 1000, odomCallback));
    }

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
