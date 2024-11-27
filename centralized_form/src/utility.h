#ifndef _UTILITY_H
#define _UTILITY_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#if 0
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
#endif
void ShowObstacleInRviz(const std::vector<std::vector<double>>& obst, const double& safety_dist, const ros::Publisher& obst_pub) {
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < obst.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacle";
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

void ShowHistoryInRviz(const std::vector<double>& x, const std::vector<double>& y, const ros::Publisher& pub) {
    static std::vector<std::vector<geometry_msgs::Point>> history_points; // 保存每辆车的历史轨迹点
    static int frame_count = 0;
    if (history_points.empty()) {
        history_points.resize(x.size());
    }

    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < x.size(); ++i) {
        // 保存当前车辆的历史轨迹点
        geometry_msgs::Point point;
        point.x = x[i];
        point.y = y[i];
        point.z = 0;
        history_points[i].push_back(point);

        // 绘制历史轨迹线段
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "history_line";
        line_strip.id = i; // 给线段一个唯一ID
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.05; // 线段宽度
        line_strip.color.r = 0.7f * (1.0 - i / x.size());
        line_strip.color.g = 0.3f;
        line_strip.color.b = 1.0f * i / x.size();
        line_strip.color.a = 0.6;

        // 添加历史轨迹点
        line_strip.points = history_points[i];

        marker_array.markers.push_back(line_strip);

        // 每隔10帧绘制一个参考球
        if (frame_count % 10 == 0) {
            visualization_msgs::Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp = ros::Time::now();
            sphere.ns = "history_sphere";
            sphere.id = i + 1000 + frame_count; // 确保ID唯一
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position.x = x[i];
            sphere.pose.position.y = y[i];
            sphere.pose.position.z = 0;
            sphere.scale.x = 0.2;
            sphere.scale.y = 0.2;
            sphere.scale.z = 0.2;
            sphere.color.r = 1.0;
            sphere.color.g = 0.0;
            sphere.color.b = 0.0;
            sphere.color.a = 0.6;

            marker_array.markers.push_back(sphere);
        }
    }

    // 发布历史轨迹和轨迹球
    pub.publish(marker_array);

    // 增加帧计数
    frame_count++;
}


void ShowVehicleInRviz(const std::vector<double> x, const std::vector<double> y, const std::vector<double> theta, const double safety_dist, const ros::Publisher& pub) {

    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < x.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "vehicle";
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
        marker.ns = "vehicle";
        marker.id = i + 1000;
        marker.type = visualization_msgs::Marker::CUBE;//柱形表示
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = safety_dist * 1;
        marker.scale.y = safety_dist * 1;
        marker.scale.z = 0.5;
        marker.color.r = 0.7f * (1.0 - i / x.size());
        marker.color.g = 0.3f;
        marker.color.b = 1.0f * i / x.size();
        marker.color.a = 0.4;
        marker.lifetime = ros::Duration();
        ma.markers.push_back(marker);
    }
    pub.publish(ma);
    ma.markers.clear();

    // 调用绘制历史轨迹的函数
    ShowHistoryInRviz(x, y, pub);
}

void ShowRefPoint(const std::vector<double> x, const std::vector<double> y, const std::vector<double> theta, const ros::Publisher& pub, const std::vector<std::vector<geometry_msgs::Point>>& historical_ref_points) {
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < x.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "reference";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;//球形表示
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = sin(theta[i] / 2.0) * 0.0;
        marker.pose.orientation.y = sin(theta[i] / 2.0) * 0.0;
        marker.pose.orientation.z = sin(theta[i] / 2.0) * 1.0;
        marker.pose.orientation.w = cos(theta[i] / 2.0);
        marker.scale.x = 0.6;
        marker.scale.y = 0.6;
        marker.scale.z = 0.6;
        marker.color.r = 0.7f * (1.0 - i / x.size());
        marker.color.g = 0.3f;
        marker.color.b = 1.0f * i / x.size();
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        ma.markers.push_back(marker);
    }
    // 绘制历史参考点
    for (int i = 0; i < historical_ref_points.size(); ++i) {
        for (int j = 0; j < historical_ref_points[i].size(); ++j) {
            visualization_msgs::Marker hist_marker;
            hist_marker.header.frame_id = "map";
            hist_marker.header.stamp = ros::Time::now();
            hist_marker.ns = "historical_reference";
            hist_marker.id = i * 1000 + j; // 确保ID唯一
            hist_marker.type = visualization_msgs::Marker::SPHERE; // 球形表示历史参考点
            hist_marker.action = visualization_msgs::Marker::ADD;
            hist_marker.pose.position = historical_ref_points[i][j];
            hist_marker.scale.x = 0.6;
            hist_marker.scale.y = 0.6;
            hist_marker.scale.z = 0.6;
            hist_marker.color.r = 0.7f * (1.0 - i / x.size());
            hist_marker.color.g = 0.3f;
            hist_marker.color.b = 1.0f * i / x.size();
            hist_marker.color.a = 0.2; // 透明度较高
            hist_marker.lifetime = ros::Duration();
            ma.markers.push_back(hist_marker);
        }
    }
    pub.publish(ma);
}

void TrajRviz(const std::vector<std::vector<std::vector<double>>> states, const double safety_dist, const ros::Publisher& pub) {
    visualization_msgs::MarkerArray ma;
    for (int i = 0;i < states.size();++i) {
        auto& state = states[i];
        for (int j = 1; j < state.size();j+=2) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectory_planned";
            marker.id = j + i * 1000;// num of cars * N
            marker.type = visualization_msgs::Marker::CUBE;
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
