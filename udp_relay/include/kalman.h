#pragma once
#include<iostream>
#include<Eigen/Eigen>
#include <ros/ros.h>
#include <deque>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class PoseDiffFilter {
public:
    PoseDiffFilter() {
        int state_dim = 3;  // 定义状态维度 (x, y, yaw)
        x_hat.setZero(state_dim);
        P.setIdentity(state_dim, state_dim);
        Q.setIdentity(state_dim, state_dim) * 0.1;  // 过程噪声
        R.setIdentity(state_dim, state_dim) * 0.1;  // 观测噪声
    }

    // 初始化滤波器
    void initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance) {
        x_hat = initial_state;
        P = initial_covariance;
    }

    // 执行EKF预测步骤
    void predict() {
        // 这里的f(x) = x，因此x_hat_minus = x_hat
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(x_hat.size(), x_hat.size());
        x_hat_minus = x_hat;  // f(x_hat)
        P_minus = F * P * F.transpose() + Q;
    }

    // 执行EKF更新步骤
    void update(const Eigen::VectorXd& z) {
        // h(x) = x，因此 H 是单位矩阵
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(x_hat.size(), x_hat.size());
        Eigen::VectorXd y = z - x_hat_minus;  // 观测残差
        Eigen::MatrixXd S = H * P_minus * H.transpose() + R;  // 残差协方差
        Eigen::MatrixXd K = P_minus * H.transpose() * S.inverse();  // 卡尔曼增益

        x_hat = x_hat_minus + K * y;  // 更新状态估计
        P = (Eigen::MatrixXd::Identity(x_hat.size(), x_hat.size()) - K * H) * P_minus;  // 更新协方差估计
    }

    // 获取当前状态估计
    Eigen::VectorXd getState() const {
        return x_hat;
    }

    // 获取当前估计协方差
    Eigen::MatrixXd getCovariance() const {
        return P;
    }

private:
    Eigen::VectorXd x_hat;       // 状态估计
    Eigen::VectorXd x_hat_minus; // 预测状态
    Eigen::MatrixXd P;           // 估计协方差
    Eigen::MatrixXd P_minus;     // 预测协方差
    Eigen::MatrixXd Q;           // 过程噪声协方差
    Eigen::MatrixXd R;           // 观测噪声协方差
};
