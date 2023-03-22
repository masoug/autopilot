#pragma once

#include <eigen3/Eigen/Core>


class Datalogger
{
public:
    Datalogger();

    void write(float timestamp);

    // variable references
    Eigen::Vector3f&
    gyro() { return m_raw_gyro; }

    Eigen::Vector3f&
    acc() { return m_raw_acc; }

    Eigen::Vector4f&
    nav() { return m_nav_state; }

    uint16_t&
    cycleSlack() { return m_cycle_slack; }

private:
    const uint8_t m_throttle;
    uint8_t m_countdown;

    static constexpr size_t FRAME_LEN = 64;
    uint8_t m_buffer[FRAME_LEN]{};

    Eigen::Vector3f m_raw_gyro;
    Eigen::Vector3f m_raw_acc;
    Eigen::Vector4f m_nav_state;
    uint16_t m_cycle_slack{};
};
