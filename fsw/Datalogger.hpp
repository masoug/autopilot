#pragma once

#include <eigen3/Eigen/Core>


class Datalogger
{
public:
    Datalogger();

    void write(
            float timestamp,
            const std::array<int16_t, 3>& raw_acc,
            const std::array<int16_t, 3>& raw_gyro,
            const Eigen::Vector4f& nav_state,
            const uint16_t& cycle_slack);

private:
    static constexpr size_t FRAME_LEN = 64;
    uint8_t m_buffer[FRAME_LEN]{};
};
