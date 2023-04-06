#pragma once

#include <eigen3/Eigen/Core>

#include "pico/stdlib.h"


class IMU
{
public:

    IMU();

    int read();

    Eigen::Vector3f
    getGyro() const {
        return GYRO_SCALE * Eigen::Vector3f(
                (float)m_raw_gyro[0],
                (float)m_raw_gyro[1],
                (float)m_raw_gyro[2]);
    }

    Eigen::Vector3f
    getAcc() const {
        return ACC_SCALE * Eigen::Vector3f(
                (float)m_raw_acc[0],
                (float)m_raw_acc[1],
                (float)m_raw_acc[2]);
    }

    float
    getTemp() const {
        return ((float)m_raw_temp / 256.0f) + 25.0f;
    }

    /**
     * Access raw unscaled values.
     */
    const std::array<int16_t, 3>&
    getRawGyro() const {
        return m_raw_gyro;
    }

    const std::array<int16_t, 3>&
    getRawAcc() const {
        return m_raw_acc;
    }

    const int16_t&
    getRawTemp() const {
        return m_raw_temp;
    }

private:
    static const uint8_t IMU_ADDR = 0x6A;
    static const uint8_t IMU_DATA_ADDR = 0x20;       ///< First data register (temperature low)
    static const uint8_t LSM6DS_CTRL1_XL = 0x10;     ///< Main accelerometer config register
    static const uint8_t LSM6DS_CTRL2_G = 0x11;      ///< Main gyro config register

    static constexpr float ACC_SCALE = 0.061f * 9.80665f / 1000.0f;
    static constexpr float GYRO_SCALE = 17.5f * 0.017453293f / 1000.0f;

    std::array<int16_t, 3> m_raw_acc{};
    std::array<int16_t, 3> m_raw_gyro{};
    int16_t m_raw_temp;
};

