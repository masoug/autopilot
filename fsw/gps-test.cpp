#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"

#include "hardware/i2c.h"

#define GPS_DEFAULT_I2C_ADDR                                                   \
  0x10 ///< The default address for I2C transport of GPS data
#define GPS_MAX_I2C_TRANSFER                                                   \
  32 ///< The max number of bytes we'll try to read at once


const uint8_t PMTK_SET_NMEA_OUTPUT_RMCGGA[] =
        "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; ///< turn on GPRMC and GPGGA
const uint8_t PMTK_SET_NMEA_OUTPUT_GGAONLY[] =
        "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; ///< turn on just the GPGGA


int main()
{
    stdio_init_all();

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool led_state = true;
    gpio_put(LED_PIN, led_state);

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // config gps
    const auto config_status = i2c_write_blocking(i2c_default, GPS_DEFAULT_I2C_ADDR,
                               PMTK_SET_NMEA_OUTPUT_GGAONLY,
                               strlen((const char*)PMTK_SET_NMEA_OUTPUT_GGAONLY),
                               true);
    printf("\nconfig_status=%d\n", config_status);

    uint8_t buffer[GPS_MAX_I2C_TRANSFER+1];
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true)
    {
        led_state = !led_state;
        gpio_put(LED_PIN, led_state);

        const auto status = i2c_read_blocking(i2c_default, GPS_DEFAULT_I2C_ADDR, buffer, GPS_MAX_I2C_TRANSFER, false);
        buffer[GPS_MAX_I2C_TRANSFER] = '\0';
        printf("%s", buffer);

        sleep_ms(500);
    }
#pragma clang diagnostic pop
    return 0;
}
