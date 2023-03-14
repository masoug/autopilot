#pragma once

#include <eigen3/Eigen/Core>


class EKF
{
public:
    explicit EKF(float gyro_noise=0.3, float acc_noise=3.0);

    void
    step(const Eigen::Vector3f& gyro,
         const Eigen::Vector3f& acc,
         float delta_t);

    const Eigen::Vector4f&
    getState() const;

private:
    void predict(const Eigen::Vector3f& gyro, float delta_t);
    void measurement();

    const float m_gyro_noise;
    const float m_gyro_noise_sq;
    const float m_acc_noise;
    const Eigen::Vector3f m_gravity;

    // attitude state quaternion & covariance
    Eigen::Vector4f m_att_q;
    Eigen::Matrix4f m_att_cov;

    // intermediate variables
    Eigen::Matrix4f m_omega;
    Eigen::Vector4f m_att_q_pred;
    Eigen::Matrix4f m_F;
    Eigen::Matrix4f m_att_q_pred_cov;
    Eigen::Vector3f m_modeled_gravity;
    Eigen::Matrix<float, 3, 4> m_H;
};

