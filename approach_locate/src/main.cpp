#include "approach_locate/locate.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "approach_locate");
    ros::NodeHandle n;
    ros::Rate loop_rate(60.0);

    ros::Publisher posPub = n.advertise<geometry_msgs::Pose>("/front_pose", 10);
    ArUcoLocation arLoc;
    if (!arLoc.init(1, 0.60, true))
        return 1;
    ArUcoPose_t pose;
    geometry_msgs::Pose pose_msg;
    while (ros::ok()) {
        if (arLoc.getArUcoPose(&pose)) {
            double rInDegree = pose.roll * 180 / 3.1416;
            double pInDegree = pose.pitch * 180 / 3.1416;
            double yInDegree = pose.yaw * 180 / 3.1416;
            pose_msg.position.x = pose.x;
            pose_msg.position.y = pose.y;
            pose_msg.position.z = pose.z;
            pose_msg.orientation.w = pose.qw;
            pose_msg.orientation.x = pose.qx;
            pose_msg.orientation.y = pose.qy;
            pose_msg.orientation.z = pose.qz;
            posPub.publish(pose_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}