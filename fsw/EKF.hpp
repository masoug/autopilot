#pragma once

#include <eigen3/Eigen/Core>


class EKF
{
public:
    explicit EKF(float gyro_noise=0.4,
                 float acc_noise=1.0);

    void stepGyrocompass(
            const Eigen::Vector3f& gyro,
            const Eigen::Vector3f& acc);

    void completeGyrocompass();

    void
    step(const Eigen::Vector3f& gyro,
         const Eigen::Vector3f& acc,
         float delta_t);

    const Eigen::Vector4f&
    getState() const;

    /**
     * Debug Fields
     */
    const Eigen::Vector3f&
    getMeasurement() const { return m_z; }

    const Eigen::Vector3f&
    getError() const { return m_error; }

    const Eigen::Vector3f&
    getModeledGravity() const { return m_modeled_gravity; }

private:
    void predict(const Eigen::Vector3f& gyro, float delta_t);
    void measurement();

    const float m_gyro_noise;
    const float m_gyro_noise_sq;
    const float m_acc_noise;
    Eigen::Vector3f m_gravity;
    float m_gravity_norm;

    Eigen::Vector3f m_gyro_bias;
    uint16_t m_cal_iterations;

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
    Eigen::Vector3f m_z;
    Eigen::Vector3f m_error;
};

