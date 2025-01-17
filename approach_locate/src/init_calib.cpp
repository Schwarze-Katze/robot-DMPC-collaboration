#include <Eigen/Eigen>
#include "approach_locate/locate.h"

nav_msgs::Odometry odom_buffer;
Eigen::Quaterniond slam_orientation_buffer;
Eigen::Vector3d slam_position_buffer;
bool has_coord_diff = false;
Eigen::Quaterniond orientation_coord_diff;
Eigen::Vector3d position_coord_diff;
ros::Publisher absOdomPub;

void SlamOdomSub(const nav_msgs::Odometry &msg)
{
    // ROS_INFO("odom received");
    odom_buffer = msg;
    slam_orientation_buffer = Eigen::Quaterniond(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
    slam_position_buffer=Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    if (has_coord_diff)
    {

        nav_msgs::Odometry abs_odom_msg;
        Eigen::Quaterniond abs_orientation = orientation_coord_diff * slam_orientation_buffer;
        Eigen::Vector3d abs_position = orientation_coord_diff * slam_position_buffer + position_coord_diff;
        abs_odom_msg.header.stamp = ros::Time::now();
        abs_odom_msg.header.frame_id = "map";
        abs_odom_msg.pose.pose.position.x = abs_position.x();
        abs_odom_msg.pose.pose.position.y = abs_position.y();
        abs_odom_msg.pose.pose.position.z = abs_position.z();
        abs_odom_msg.pose.pose.orientation.w = abs_orientation.w();
        abs_odom_msg.pose.pose.orientation.x = abs_orientation.x();
        abs_odom_msg.pose.pose.orientation.y = abs_orientation.y();
        abs_odom_msg.pose.pose.orientation.z = abs_orientation.z();
        std::cout << abs_position << std::endl << abs_orientation.coeffs()<< std::endl;
        absOdomPub.publish(abs_odom_msg);
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "init_locate");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(60.0);

    ros::Publisher selfPosePub = n.advertise<geometry_msgs::PoseStamped>("/self_abs_pose", 10);
    ros::Subscriber slamOdomSub = n.subscribe("/Odometry", 10,SlamOdomSub);
    absOdomPub = n.advertise<nav_msgs::Odometry>("/Odometry_abs", 10);

    // 读取二维码位姿参数
    double qr_x, qr_y, qr_z, qr_qx, qr_qy, qr_qz, qr_qw;
    n.getParam("qr_x", qr_x);
    n.getParam("qr_y", qr_y);
    n.getParam("qr_z", qr_z);
    n.getParam("qr_qx", qr_qx);
    n.getParam("qr_qy", qr_qy);
    n.getParam("qr_qz", qr_qz);
    n.getParam("qr_qw", qr_qw);

    Eigen::Quaterniond aruco_abs_orientation(qr_qw, qr_qx, qr_qy, qr_qz);
    Eigen::Vector3d aruco_abs_position(qr_x, qr_y, qr_z);

    ArUcoLocation arLoc;
    if (!arLoc.init(0, true))
        return 1;

    std::vector<ArUcoPose_t> posevec;
    geometry_msgs::PoseStamped pose_msg, rotate_pose, self_pose;
    bool has_mark_pose = false;

    while (ros::ok()) {
#ifdef DEBUG_NO_VIDEO
        if (true) {
            posevec.clear();
            ArUcoPose_t pose1;
            pose1.id = 101;
            pose1.qx = pose1.qy = pose1.qz = 0;
            pose1.qw = 1;
            posevec.push_back(pose1);
#else 
        if (arLoc.getArUcoPose(posevec)) {
#endif
            for (auto& pose : posevec) {
                if (pose.x * pose.x + pose.y * pose.y > 1.0) {
                    ROS_INFO("Skip marker distance > 1m");
                    continue;
                }
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.pose.position.x = pose.x;
                pose_msg.pose.position.y = pose.y;
                pose_msg.pose.position.z = pose.z;
                pose_msg.pose.orientation.w = pose.qw;
                pose_msg.pose.orientation.x = pose.qx;
                pose_msg.pose.orientation.y = pose.qy;
                pose_msg.pose.orientation.z = pose.qz;

                if (pose.id == 101) {
                    pose_msg.header.frame_id = "mark_link";
                    rotate_pose = pose_msg;
                    has_mark_pose = true;
                }
                else {
                    continue;
                }

                if (has_mark_pose) {
                    has_mark_pose = false;

                    // 计算自身坐标
                    Eigen::Quaterniond aruco_relative_orientation(rotate_pose.pose.orientation.w, rotate_pose.pose.orientation.x, rotate_pose.pose.orientation.y, rotate_pose.pose.orientation.z);
                    Eigen::Vector3d aruco_relative_position(rotate_pose.pose.position.x, rotate_pose.pose.position.y, rotate_pose.pose.position.z);

                    Eigen::Quaterniond self_orientation = aruco_abs_orientation * aruco_relative_orientation.inverse();
                    Eigen::Vector3d self_position = aruco_abs_position - self_orientation * aruco_relative_position;

                    // 计算LiDAR绝对坐标
                    Eigen::Quaterniond aruco_to_lidar_orientation(Eigen::AngleAxisd(-20 * M_PI / 180, Eigen::Vector3d::UnitY()));
                    Eigen::Vector3d aruco_to_lidar_position(-0.29, 0.0, 0.06);
                    self_position += aruco_to_lidar_position;
                    self_orientation *= aruco_to_lidar_orientation;

                    // 发布自身位姿
                    self_pose.header.stamp = ros::Time::now();
                    self_pose.header.frame_id = "map";
                    self_pose.pose.position.x = self_position.x();
                    self_pose.pose.position.y = self_position.y();
                    self_pose.pose.position.z = self_position.z();
                    self_pose.pose.orientation.w = self_orientation.w();
                    self_pose.pose.orientation.x = self_orientation.x();
                    self_pose.pose.orientation.y = self_orientation.y();
                    self_pose.pose.orientation.z = self_orientation.z();

                    orientation_coord_diff = self_orientation * slam_orientation_buffer.inverse();
                    position_coord_diff = self_position - self_orientation * slam_position_buffer;
                    has_coord_diff = true;

                    selfPosePub.publish(self_pose);

                    
                }
                posevec.clear();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
