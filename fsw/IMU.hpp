#pragma once

#include <eigen3/Eigen/Core>

#include "pico/stdlib.h"


class IMU
{
public:
    static const uint8_t IMU_ADDR = 0x6A;
    static const uint8_t IMU_DATA_ADDR = 0x20;       ///< First data register (temperature low)
    static const uint8_t LSM6DS_CTRL1_XL = 0x10;     ///< Main accelerometer config register
    static const uint8_t LSM6DS_CTRL2_G = 0x11;      ///< Main gyro config register

    IMU();

    int read();

    const Eigen::Vector3f&
    getGyro() const {
        return m_gyro;
    }

    const Eigen::Vector3f&
    getAcc() const {
        return m_acc;
    }

    float
    getTemp() const {
        return m_temperature;
    }

private:
    Eigen::Vector3f m_acc;
    Eigen::Vector3f m_gyro;
    float m_temperature{};
};

