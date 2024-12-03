#include "approach_locate/locate.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "approach_locate");
    ros::NodeHandle n;
    ros::Rate loop_rate(60.0);

    ros::Publisher rotatePosPub = n.advertise<geometry_msgs::PoseStamped>("/front_rotate_pose", 10);
    ros::Publisher bodyPosPub = n.advertise<geometry_msgs::PoseStamped>("/front_body_pose", 10);
    ArUcoLocation arLoc;
    if (!arLoc.init(0, true))
        return 1;
    std::vector<ArUcoPose_t> posevec;
    geometry_msgs::PoseStamped pose_msg;
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
                }
                else if (pose.id == 240) {
                    pose_msg.header.frame_id = "body_link";
                    bodyPosPub.publish(pose_msg);
                }
                else {
                    continue;
                }
            }
            posevec.clear();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}