#include "EKF.hpp"

#include <eigen3/Eigen/LU>


namespace {

void predict_jacobian(
        Eigen::Matrix4f& du,            // output variable `F`
        const Eigen::Vector3f& RHS1,    // angular rate `ω`
        const float RHS2                // timestep `Δt`
)
{
    du(0, 0) = 1;
    du(0, 1) = 0.5 * RHS2 * RHS1[0];
    du(0, 2) = 0.5 * RHS2 * RHS1[1];
    du(0, 3) = 0.5 * RHS2 * RHS1[2];

    du(1, 0) = -0.5 * RHS2 * RHS1[0];
    du(1, 1) = 1;
    du(1, 2) = -0.5 * RHS2 * RHS1[2];
    du(1, 3) = 0.5 * RHS2 * RHS1[1];

    du(2, 0) = -0.5 * RHS2 * RHS1[1];
    du(2, 1) = 0.5 * RHS2 * RHS1[2];
    du(2, 2) = 1;
    du(2, 3) = -0.5 * RHS2 * RHS1[0];

    du(3, 0) = -0.5 * RHS2 * RHS1[2];
    du(3, 1) = -0.5 * RHS2 * RHS1[1];
    du(3, 2) = 0.5 * RHS2 * RHS1[0];
    du(3, 3) = 1;
}

inline float pow(const float& base, const int exp)
{
    float result = 1.0;
    for (int i = 0; i < exp; i++)
    {
        result *= base;
    }
    return result;
}

void quat2mat(
        Eigen::Matrix3f& du,         // output variable rotation matrix
        const Eigen::Vector4f& RHS1  // quaternion `q`
)
{
    du(0, 0) = 1.0 + -2.0 * (pow(RHS1[2], 2) + pow(RHS1[3], 2));
    du(0, 1) = 2.0 * (RHS1[0] * RHS1[3] + RHS1[1] * RHS1[2]);
    du(0, 2) = 2.0 * (RHS1[1] * RHS1[3] + -1 * RHS1[0] * RHS1[2]);

    du(1, 0) = 2.0 * (RHS1[1] * RHS1[2] + -1 * RHS1[0] * RHS1[3]);
    du(1, 1) = 1.0 + -2.0 * (pow(RHS1[1], 2) + pow(RHS1[3], 2));
    du(1, 2) = 2.0 * (RHS1[0] * RHS1[1] + RHS1[2] * RHS1[3]);

    du(2, 0) = 2.0 * (RHS1[0] * RHS1[2] + RHS1[1] * RHS1[3]);
    du(2, 1) = 2.0 * (RHS1[2] * RHS1[3] + -1 * RHS1[0] * RHS1[1]);
    du(2, 2) = 1.0 + -2.0 * (pow(RHS1[1], 2) + pow(RHS1[2], 2));
}

void measurement_jacobian(
        Eigen::Matrix<float, 3, 4>& du,     // output variable `H`
        const Eigen::Vector4f& RHS1,        // quaternion `q`
        const Eigen::Vector3f& RHS2         // gravity vector `g`
)
{
    du(0, 0) = -2.0 * RHS2[2] * RHS1[2] + 2.0 * RHS2[1] * RHS1[3];
    du(0, 1) = -2.0 * RHS2[0] * RHS1[3] + 2.0 * RHS2[2] * RHS1[1];
    du(0, 2) = -2.0 * RHS2[1] * RHS1[1] + 2.0 * RHS2[0] * RHS1[2];
    du(0, 3) = 2.0 * RHS2[1] * RHS1[2] + 2.0 * RHS2[2] * RHS1[3];

    du(1, 0) = -4.0 * RHS2[1] * RHS1[1] + 2.0 * RHS2[2] * RHS1[0] + 2.0 * RHS2[0] * RHS1[2];
    du(1, 1) = 2.0 * RHS2[0] * RHS1[3] + -2.0 * RHS2[1] * RHS1[0] + -4.0 * RHS2[2] * RHS1[1];
    du(1, 2) = 2.0 * RHS2[1] * RHS1[1] + -2.0 * RHS2[2] * RHS1[0] + -4.0 * RHS2[0] * RHS1[2];
    du(1, 3) = 2.0 * RHS2[0] * RHS1[1] + 2.0 * RHS2[2] * RHS1[3];

    du(2, 0) = 2.0 * RHS2[0] * RHS1[0] + -4.0 * RHS2[2] * RHS1[2] + 2.0 * RHS2[1] * RHS1[3];
    du(2, 1) = -4.0 * RHS2[0] * RHS1[3] + 2.0 * RHS2[1] * RHS1[0] + 2.0 * RHS2[2] * RHS1[1];
    du(2, 2) = -2.0 * RHS2[0] * RHS1[0] + 2.0 * RHS2[2] * RHS1[2] + -4.0 * RHS2[1] * RHS1[3];
    du(2, 3) = 2.0 * RHS2[0] * RHS1[1] + 2.0 * RHS2[1] * RHS1[2];
}

}

EKF::EKF(const float gyro_noise, const float acc_noise)
    : m_gyro_noise(gyro_noise)
    , m_gyro_noise_sq(gyro_noise*gyro_noise)
    , m_acc_noise(acc_noise)
    , m_gravity(0.0, 0.0, -1.0)
    , m_att_q(1.0, 0.0, 0.0, 0.0)
    , m_att_cov(Eigen::Matrix4f::Identity())
{
}

const Eigen::Vector4f&
EKF::getState() const {
    return m_att_q;
}

void
EKF::predict(const Eigen::Vector3f &gyro, const float delta_t)
{
    // Big idea: Predict the next state by integrating angular rates `ω` by `Δt`,
    // and adding that to current_state `q`.
    m_omega <<    0.0f, -gyro[0], -gyro[1], -gyro[2],
               gyro[0],     0.0f,  gyro[2], -gyro[1],
               gyro[1], -gyro[2],     0.0f,  gyro[0],
               gyro[2],  gyro[1], -gyro[0],     0.0f;

    // Here we use a linearized approximation of the Euler-Rodrigues rotation formula
    // to compute the predicted quaternion.
    m_att_q_pred = (Eigen::Matrix4f::Identity() + 0.5*delta_t*m_omega)*m_att_q;

    // prediction jacobian
    predict_jacobian(m_F, gyro, delta_t);

    // compute approximated matrix exponential jacobian w.r.t. ω
    Eigen::Matrix<float, 4, 3> W;
    W << -m_att_q_pred[1], -m_att_q_pred[2], -m_att_q_pred[3],
          m_att_q_pred[0], -m_att_q_pred[3],  m_att_q_pred[2],
          m_att_q_pred[3],  m_att_q_pred[0], -m_att_q_pred[1],
         -m_att_q_pred[2],  m_att_q_pred[1],  m_att_q_pred[0];
    W *= 0.5*delta_t;

    // compute process noise
    const Eigen::Matrix4f Q = m_gyro_noise_sq*W*W.transpose();

    // compute prediction noise
    m_att_q_pred_cov = m_F*m_att_cov*m_F.transpose() + Q;
}

void EKF::measurement()
{
    // Convert quaternion to rotation matrix
    Eigen::Matrix3f rot;
    quat2mat(rot, m_att_q_pred);

    // apply rotation to calibrated gravity vector
    m_modeled_gravity = rot.transpose()*m_gravity;

    // compute measurement model jacobian
    measurement_jacobian(m_H, m_att_q_pred, m_gravity);
}

void
EKF::step(const Eigen::Vector3f& gyro,
          const Eigen::Vector3f& acc,
          const float delta_t)
{
    // Predict!
    predict(gyro, delta_t);

    // update modeled gravity
    measurement();

    // Normalize gravity vector since we only care about direction
    const Eigen::Vector3f z = acc.normalized();

    // observation error
    const Eigen::Vector3f error = z - m_modeled_gravity;

    // observation noise
    const Eigen::Matrix3f R = m_acc_noise*Eigen::Matrix3f::Identity();

    // kalman gain
    const Eigen::Matrix3f S = m_H*m_att_q_pred_cov*m_H.transpose() + R;
    const Eigen::Matrix<float, 4, 3> K = m_att_q_pred_cov*m_H.transpose()*S.inverse();

    // apply corrections
    m_att_q = (m_att_q_pred + K*error).normalized();

    // update covariance
    m_att_cov = (Eigen::Matrix4f::Identity()-K*m_H)*m_att_q_pred_cov;
}
