/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include "pico/stdlib.h"

#include "Datalogger.hpp"
#include "EKF.hpp"
#include "IMU.hpp"
#include "LoopRate.hpp"


int main()
{
    stdio_init_all();

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool led_state = true;
    gpio_put(LED_PIN, led_state);

    // give some time for the peripherals to warm up
    sleep_ms(3000);

    // navigation subsystems
    IMU imu;
    EKF ekf;

    // logging & debugging systems
    Datalogger dlog;

    LoopRate rate(10);
    uint8_t cycle_counter = 0;
    float prev_ts = 0.0;
    int64_t cycle_slack = 0;
    while (true)
    {
        const float ts = to_us_since_boot(get_absolute_time()) * 1e-6f;
        const float delta_t = std::max(0.005f, ts - prev_ts);
        prev_ts = ts;

        // Read sensor data
        const auto imu_read_status = imu.read();

        // Update navigation algorithms
        const auto gyro = (imu_read_status == 14) ? imu.getGyro() : Eigen::Vector3f::Zero();
        const auto acc = (imu_read_status == 14) ? imu.getAcc() : Eigen::Vector3f::Zero();
        ekf.step(gyro, acc, delta_t);
        const auto nav = ekf.getState();

        // TODO Guidance
        // TODO Control

        // LED heartbeat indicator
        gpio_put(LED_PIN, led_state);
        if (cycle_counter == 50) {
            led_state = !led_state;
            cycle_counter = 0;
        }

        // Output telemetry
        dlog.write(ts,
                   imu.getRawAcc(),
                   imu.getRawGyro(),
                   nav,
                   cycle_slack);

        cycle_slack = rate.wait();
        cycle_counter++;
    }
    return 0;
}
