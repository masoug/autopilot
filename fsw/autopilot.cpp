/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include "pico/stdlib.h"

#include "EKF.hpp"
#include "IMU.hpp"
#include "LoopRate.hpp"


int main()
{
    stdio_init_all();

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // navigation subsystems
    IMU imu;
    EKF ekf;

    LoopRate rate(10);
    int64_t cycle_slack = 0;
    bool led_state = false;
    uint8_t cycle_counter = 0;
    while (true)
    {
        const float ts = to_us_since_boot(get_absolute_time()) * 1e-6f;

        // Read sensor data
        const auto imu_read_status = imu.read();
        if (imu_read_status != 14)
        {
            break;
        }

        // Update navigation algorithms
        const auto& gyro = imu.getGyro();
        const auto& acc = imu.getAcc();
        ekf.step(gyro, acc, 0.01);
        const auto& nav = ekf.getState();

        // TODO Guidance
        // TODO Control

        // LED heartbeat indicator
        gpio_put(LED_PIN, led_state);
        if (cycle_counter == 50) {
            led_state = !led_state;
            cycle_counter = 0;
        }

        // Output telemetry
        printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
               ts,
               gyro[0], gyro[1], gyro[2],
               acc[0], acc[1], acc[2],
               nav[0], nav[1], nav[2], nav[3],
               cycle_slack);

        cycle_slack = rate.wait();
        cycle_counter++;
    }
    return 0;
}
