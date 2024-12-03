#include "approach_locate/locate.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "approach_locate");
    ros::NodeHandle n;
    ros::Rate loop_rate(60.0);

    ros::Publisher rotatePosPub = n.advertise<geometry_msgs::PoseStamped>("/front_rotate_pose", 10);
    ros::Publisher bodyPosPub = n.advertise<geometry_msgs::PoseStamped>("/front_body_pose", 10);
    ros::Publisher anglePub = n.advertise<std_msgs::Float64>("/rotate_angle", 10);

    ArUcoLocation arLoc;
    if (!arLoc.init(0, true))
        return 1;
    std::vector<ArUcoPose_t> posevec;
    geometry_msgs::PoseStamped pose_msg, rotate_pose, body_pose;
    bool has_rotate_pose = false, has_body_pose = false;

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
                if (pose.id == 203) {
                    pose_msg.header.frame_id = "rotate_link";
                    rotatePosPub.publish(pose_msg);
                    rotate_pose = pose_msg;
                    has_rotate_pose = true;
                }
                else if (pose.id == 240) {
                    pose_msg.header.frame_id = "body_link";
                    bodyPosPub.publish(pose_msg);
                    body_pose = pose_msg;
                    has_body_pose = true;
                }
                else {
                    continue;
                }

                if (has_rotate_pose && has_body_pose) {
                    const auto& q1 = rotate_pose.pose.orientation;
                    const auto& q2 = body_pose.pose.orientation;

                    double dot_product = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
                    if (dot_product > 1.0) {
                        dot_product = 1.0;
                    }
                    else if (dot_product < -1.0) {
                        dot_product = -1.0;
                    }

                    double angle = 2.0 * acos(dot_product);

                    // 发布夹角
                    std_msgs::Float64 angle_msg;
                    angle_msg.data = angle;
                    anglePub.publish(angle_msg);
                }
                posevec.clear();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}