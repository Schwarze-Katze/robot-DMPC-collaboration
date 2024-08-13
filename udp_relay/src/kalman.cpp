#include "kalman.h"

Kalman::Kalman() {
    {
        m_StateSize = 4;//statesize
        m_MeaSize = 4;//measize
        m_USize = 4;//usize
        last_time = ros::Time(0);
        if (m_StateSize == 0 && m_MeaSize == 0) {
            ROS_INFO("Init Kalman Filter Class...");
        }

        // m_x.resize(statesize);
        m_x.setZero();

        // m_u.resize(usize);
        m_u.setZero();

        // m_z.resize(measize);
        m_z.setZero();

        // m_A.resize(statesize, statesize);
        m_A.setIdentity();

        // m_B.resize(statesize, usize);
        m_B.setZero();

        // m_P.resize(statesize, statesize);
        m_P.setIdentity();

        // m_H.resize(measize, statesize);
        m_H.setZero();

        // m_R.resize(measize, measize);
        m_R.setZero();

        // m_Q.resize(statesize, statesize);
        m_Q.setZero();

        // m_iden_mat.resize(statesize, statesize);
        m_iden_mat.setIdentity();

        timestamp = timestamp_last = 0.0;
    }
}

void Kalman::Init_Par(Eigen::Vector4d x, Eigen::Matrix4d P, Eigen::Matrix4d R, Eigen::Matrix4d Q, Eigen::Matrix4d A, Eigen::Matrix4d B, Eigen::Matrix4d H, Eigen::Vector4d u) {
    m_x = x;
    m_P = P;
    m_R = R;
    m_Q = Q;
    m_A = A;
    m_B = B;
    m_H = H;
    m_u = u;
}

// 得到先验x
void Kalman::Predict_State() {
    m_x = m_A * m_x + m_B * m_u;
}

// 得到先验P
void Kalman::Predict_Cov() {
    m_P = m_A * m_P * m_A.transpose().eval() + m_Q;
}

// 得到观测和预测先验的差值，用于计算x后验
Eigen::Vector4d Kalman::Mea_Resd() {
    return m_z - m_H * m_x;
}

// 得到卡尔曼增益
Eigen::Matrix4d Kalman::Cal_Gain() {
    return m_P * m_H.transpose() * (m_H * m_P * m_H.transpose() + m_R).inverse();
}

// 得到后验x
void Kalman::Update_State() {
    m_x = m_x + Cal_Gain() * Mea_Resd();
}

// 计算后验p
void Kalman::Update_Cov() {
    m_P = (m_iden_mat - Cal_Gain() * m_H) * m_P;
}

