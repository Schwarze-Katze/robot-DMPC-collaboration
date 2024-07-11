#ifndef _UTILITY_H
#define _UTILITY_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
void ShowVehicleInRviz(const double& x, const double& y, const double& theta, const ros::Publisher& vehicle_pub) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = sin(theta / 2.0) * 0.0;
    marker.pose.orientation.y = sin(theta / 2.0) * 0.0;
    marker.pose.orientation.z = sin(theta / 2.0) * 1.0;
    marker.pose.orientation.w = cos(theta / 2.0);
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    vehicle_pub.publish(marker);
};
void ShowObstacleInRviz(const std::vector<std::vector<double>>& obst, const double& safety_dist, const ros::Publisher& obst_pub) {
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < obst.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = obst[i][0];
        marker.pose.position.y = obst[i][1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = safety_dist * 2.0;
        marker.scale.y = safety_dist * 2.0;
        marker.scale.z = 0.1;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        ma.markers.push_back(marker);
    }
    obst_pub.publish(ma);
    ma.markers.clear();
};
void ShowVehicleInRviz(const std::vector<double> x, const std::vector<double> y, const std::vector<double> theta, const double safety_dist, const ros::Publisher& pub) {

    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < x.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i + 100;
        marker.type = visualization_msgs::Marker::ARROW;//箭头表示
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = sin(theta[i] / 2.0) * 0.0;
        marker.pose.orientation.y = sin(theta[i] / 2.0) * 0.0;
        marker.pose.orientation.z = sin(theta[i] / 2.0) * 1.0;
        marker.pose.orientation.w = cos(theta[i] / 2.0);
        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.7f * (1.0 - i / x.size());
        marker.color.g = 0.3f;
        marker.color.b = 1.0f * i / x.size();
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        ma.markers.push_back(marker);
    }
    pub.publish(ma);
    ma.markers.clear();

    for (int i = 0; i < x.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i + 1000;
        marker.type = visualization_msgs::Marker::CYLINDER;//圆柱状表示
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = safety_dist * 1.0;
        marker.scale.y = safety_dist * 1.0;
        marker.scale.z = 0.1;
        marker.color.r = 0.7f * (1.0 - i / x.size());
        marker.color.g = 0.3f;
        marker.color.b = 1.0f * i / x.size();
        marker.color.a = 0.4;
        marker.lifetime = ros::Duration();
        ma.markers.push_back(marker);
    }
    pub.publish(ma);
    ma.markers.clear();
}

void ShowRefPoint(const std::vector<double> x, const std::vector<double> y, const std::vector<double> theta, const ros::Publisher& pub) {
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < x.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i + 2000;
        marker.type = visualization_msgs::Marker::SPHERE;//球形表示
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = sin(theta[i] / 2.0) * 0.0;
        marker.pose.orientation.y = sin(theta[i] / 2.0) * 0.0;
        marker.pose.orientation.z = sin(theta[i] / 2.0) * 1.0;
        marker.pose.orientation.w = cos(theta[i] / 2.0);
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 0.7f * (1.0 - i / x.size());
        marker.color.g = 0.3f;
        marker.color.b = 1.0f * i / x.size();
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        ma.markers.push_back(marker);
    }
    pub.publish(ma);
}

void TrajRviz(const std::vector<std::vector<std::vector<double>>> states, const double safety_dist, const ros::Publisher& pub) {
    visualization_msgs::MarkerArray ma;
    for (int i = 0;i < states.size();++i) {
        auto& state = states[i];
        for (int j = 1; j < state.size();j++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = j + i * 1000 + 3000;// num of cars * N
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = state[j][0];
            marker.pose.position.y = state[j][1];
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = safety_dist * 0.5;
            marker.scale.y = safety_dist * 0.5;
            marker.scale.z = 0.1;
            marker.color.r = 0.7f;
            marker.color.g = 0.3f;
            marker.color.b = 0.3f;
            marker.color.a = 0.1;
            marker.lifetime = ros::Duration();
            ma.markers.push_back(marker);
        }
    }
    pub.publish(ma);
}

double dis2d(double x1, double y1, double x2, double y2) {
    auto dx = x1 - x2;
    auto dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
};

#endif 
