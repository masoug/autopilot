#include "Datalogger.hpp"

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "utils.hpp"


#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1


Datalogger::Datalogger()
{
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    memset(m_buffer, 0, FRAME_LEN);
}

void Datalogger::write(
        const float timestamp,
        const std::array<int16_t, 3>& raw_acc,
        const std::array<int16_t, 3>& raw_gyro,
        const Eigen::Vector4f& nav_state,
        const uint16_t& cycle_slack)
{
//    snprintf(buffer, FRAME_LEN,
//             "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
//             timestamp,
//             m_raw_gyro[0], m_raw_gyro[1], m_raw_gyro[2],
//             m_raw_acc[0], m_raw_acc[1], m_raw_acc[2],
//             m_nav_state[0], m_nav_state[1], m_nav_state[2], m_nav_state[3],
//             m_cycle_slack);

    // start writing the fields
    size_t offset = 0;
    memcpy(m_buffer + offset, &timestamp, 4);
    offset += 4;

    memcpy(m_buffer+offset, raw_gyro.data(), 6);
    offset += 6;

    memcpy(m_buffer+offset, raw_acc.data(), 6);
    offset += 6;

    for (int i = 0; i < 4; i++)
    {
        memcpy(m_buffer + offset, &nav_state[i], 4);
        offset += 4;
    }

    memcpy(m_buffer + offset, &cycle_slack, 2);
    offset += 2;

    // Add a crc-8 checksum at the end
    const auto checksum = utils::crc8(m_buffer, offset);
    memcpy(m_buffer + offset, &checksum, 1);
    offset += 1;

    uart_write_blocking(UART_ID, m_buffer, offset);
}
