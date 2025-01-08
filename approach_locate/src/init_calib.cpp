#include <Eigen/Eigen>
#include "approach_locate/locate.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "init_locate");
    ros::NodeHandle n;
    ros::Rate loop_rate(60.0);

    ros::Publisher selfPosePub = n.advertise<geometry_msgs::PoseStamped>("/self_pose", 10);

    // 读取二维码位姿参数
    double qr_x, qr_y, qr_z, qr_qx, qr_qy, qr_qz, qr_qw;
    n.getParam("/qr_x", qr_x);
    n.getParam("/qr_y", qr_y);
    n.getParam("/qr_z", qr_z);
    n.getParam("/qr_qx", qr_qx);
    n.getParam("/qr_qy", qr_qy);
    n.getParam("/qr_qz", qr_qz);
    n.getParam("/qr_qw", qr_qw);

    Eigen::Quaterniond qr_orientation(qr_qw, qr_qx, qr_qy, qr_qz);
    Eigen::Vector3d qr_position(qr_x, qr_y, qr_z);

    ArUcoLocation arLoc;
    if (!arLoc.init(0, true))
        return 1;

    std::vector<ArUcoPose_t> posevec;
    geometry_msgs::PoseStamped pose_msg, rotate_pose, body_pose, self_pose;
    bool has_mark_pose = false;

    while (ros::ok()) {
        if (arLoc.getArUcoPose(posevec)) {
            for (auto& pose : posevec) {
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
                    const auto& q1 = rotate_pose.pose.orientation;
                    const auto& q2 = body_pose.pose.orientation;
                    has_mark_pose = false;

                    // 计算自身坐标
                    Eigen::Quaterniond aruco_orientation(rotate_pose.pose.orientation.w, rotate_pose.pose.orientation.x, rotate_pose.pose.orientation.y, rotate_pose.pose.orientation.z);
                    Eigen::Vector3d aruco_position(rotate_pose.pose.position.x, rotate_pose.pose.position.y, rotate_pose.pose.position.z);

                    Eigen::Quaterniond self_orientation = qr_orientation * aruco_orientation.inverse();
                    Eigen::Vector3d self_position = qr_position - self_orientation * aruco_position;

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

                    selfPosePub.publish(self_pose);
                }
                posevec.clear();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
