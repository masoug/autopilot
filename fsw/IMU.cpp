#include "IMU.hpp"

#include "hardware/i2c.h"


IMU::IMU()
{
    // This IMU will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    uint8_t command[2];

    // configure accelerometer (104hz, 2g)
    command[0] = LSM6DS_CTRL1_XL;
    command[1] = 0x40;
    i2c_write_blocking(i2c_default, IMU_ADDR, command, 2, true);

    // configure gyro (104hz, 500dps)
    command[0] = LSM6DS_CTRL2_G;
    command[1] = 0x44;
    i2c_write_blocking(i2c_default, IMU_ADDR, command, 2, true);
}

int
IMU::read()
{
    uint8_t buffer[14];
    const uint8_t imu_data_addr = IMU_DATA_ADDR;
    i2c_write_blocking(i2c_default, IMU_ADDR, &imu_data_addr, 1, true);
    const int status = i2c_read_blocking(i2c_default, IMU_ADDR, buffer, 14, false);
    if (status != 14)
    {
        return status;
    }

    // process the raw data
    int16_t rawTemp = buffer[1] << 8 | buffer[0];
    m_temperature = ((float)rawTemp / 256.0f) + 25.0f;

    int16_t rawGyroX = buffer[3] << 8 | buffer[2];
    int16_t rawGyroY = buffer[5] << 8 | buffer[4];
    int16_t rawGyroZ = buffer[7] << 8 | buffer[6];

    int16_t rawAccX = buffer[9] << 8 | buffer[8];
    int16_t rawAccY = buffer[11] << 8 | buffer[10];
    int16_t rawAccZ = buffer[13] << 8 | buffer[12];

    const float gyro_scale = 17.5f; // range is in milli-dps per bit!
    m_gyro[0] = (float)rawGyroX * gyro_scale * 0.017453293f / 1000.0f;
    m_gyro[1] = (float)rawGyroY * gyro_scale * 0.017453293f / 1000.0f;
    m_gyro[2] = (float)rawGyroZ * gyro_scale * 0.017453293f / 1000.0f;

    const float accel_scale = 0.061f; // range is in milli-g per bit!
    m_acc[0] = (float)rawAccX * accel_scale * 9.80665f / 1000.0f;
    m_acc[1] = (float)rawAccY * accel_scale * 9.80665f / 1000.0f;
    m_acc[2] = (float)rawAccZ * accel_scale * 9.80665f / 1000.0f;

    return status;
}

/**
 * Recycle Bin
 */
//#define IMU_ADDR        0x6A
//#define IMU_DATA_ADDR   0x20     ///< First data register (temperature low)
//#define LSM6DS_CTRL1_XL 0x10     ///< Main accelerometer config register
//#define LSM6DS_CTRL2_G 0x11      ///< Main gyro config register
//
//
//void
//setup_imu()
//{
//    // This IMU will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
//    i2c_init(i2c_default, 100 * 1000);
//    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
//    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
//    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
//    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
//    uint8_t command[2];
//
//    // configure accelerometer (104hz, 2g)
//    command[0] = LSM6DS_CTRL1_XL;
//    command[1] = 0x40;
//    i2c_write_blocking(i2c_default, IMU_ADDR, command, 2, true);
//
//    // configure gyro (104hz, 500dps)
//    command[0] = LSM6DS_CTRL2_G;
//    command[1] = 0x44;
//    i2c_write_blocking(i2c_default, IMU_ADDR, command, 2, true);
//}
//
//int
//read_imu(float acc[3], float gyro[3], float* temperature)
//{
//    uint8_t buffer[14];
//    const uint8_t imu_data_addr = IMU_DATA_ADDR;
//    i2c_write_blocking(i2c_default, IMU_ADDR, &imu_data_addr, 1, true);
//    const int status = i2c_read_blocking(i2c_default, IMU_ADDR, buffer, 14, false);
//    if (status != 14)
//    {
//        return status;
//    }
//
//    // process the raw data
//    int16_t rawTemp = buffer[1] << 8 | buffer[0];
//    *temperature = ((float)rawTemp / 256.0f) + 25.0f;
//
//    int16_t rawGyroX = buffer[3] << 8 | buffer[2];
//    int16_t rawGyroY = buffer[5] << 8 | buffer[4];
//    int16_t rawGyroZ = buffer[7] << 8 | buffer[6];
//
//    int16_t rawAccX = buffer[9] << 8 | buffer[8];
//    int16_t rawAccY = buffer[11] << 8 | buffer[10];
//    int16_t rawAccZ = buffer[13] << 8 | buffer[12];
//
//    const float gyro_scale = 17.5f; // range is in milli-dps per bit!
//    gyro[0] = (float)rawGyroX * gyro_scale * 0.017453293f / 1000.0f;
//    gyro[1] = (float)rawGyroY * gyro_scale * 0.017453293f / 1000.0f;
//    gyro[2] = (float)rawGyroZ * gyro_scale * 0.017453293f / 1000.0f;
//
//    const float accel_scale = 0.061f; // range is in milli-g per bit!
//    acc[0] = (float)rawAccX * accel_scale * 9.80665f / 1000.0f;
//    acc[1] = (float)rawAccY * accel_scale * 9.80665f / 1000.0f;
//    acc[2] = (float)rawAccZ * accel_scale * 9.80665f / 1000.0f;
//
//    return status;
//}