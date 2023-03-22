#include "Datalogger.hpp"

#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1


Datalogger::Datalogger()
    : m_throttle(1)
    , m_countdown(m_throttle)
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

void Datalogger::write(const float timestamp)
{
    if (m_countdown > 0)
    {
        m_countdown--;
        return;
    }
    else
    {
        m_countdown = m_throttle;
    }

//    snprintf(buffer, FRAME_LEN,
//             "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
//             timestamp,
//             m_raw_gyro[0], m_raw_gyro[1], m_raw_gyro[2],
//             m_raw_acc[0], m_raw_acc[1], m_raw_acc[2],
//             m_nav_state[0], m_nav_state[1], m_nav_state[2], m_nav_state[3],
//             m_cycle_slack);

    size_t offset = 0;
    memcpy(m_buffer + offset, &timestamp, 4);
    offset += 4;

    for (int i = 0; i < 3; i++)
    {
        memcpy(m_buffer + offset, &m_raw_gyro[i], 4);
        offset += 4;
    }

    for (int i = 0; i < 3; i++)
    {
        memcpy(m_buffer + offset, &m_raw_acc[i], 4);
        offset += 4;
    }

    for (int i = 0; i < 4; i++)
    {
        memcpy(m_buffer + offset, &m_nav_state[i], 4);
        offset += 4;
    }

    memcpy(m_buffer + offset, &m_cycle_slack, 2);
    offset += 2;

    uart_write_blocking(UART_ID, m_buffer, offset);
}
