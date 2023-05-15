#pragma once

#include <eigen3/Eigen/Core>

#include "pico/stdlib.h"


namespace utils
{

uint8_t
crc8(const uint8_t* buf, size_t len);

class LocalNED
{
public:
    static
    Eigen::Vector3d
    lla2ecef(float lat, float lon, float alt);

    LocalNED(float lat, float lon, float alt);

    Eigen::Vector3f
    lla2ned(float lat, float lon, float alt) const;

private:
    static
    Eigen::Matrix3d
    rotNED(float lat, float lon);

    const Eigen::Vector3d m_origin;
    const Eigen::Matrix3d m_R;
};

}
