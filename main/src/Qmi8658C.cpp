#include "Qmi8658C.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include <cstring>
#include <tgmath.h>

namespace Hilton {

esp_err_t Qmi8658C::writeReg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(
        i2cPort, QMI8658C_SLAVE_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

esp_err_t Qmi8658C::readReg(uint8_t reg, uint8_t *data)
{
    return i2c_master_write_read_device(
        i2cPort, QMI8658C_SLAVE_ADDR, &reg, 1, data, 1, pdMS_TO_TICKS(100));
}

esp_err_t Qmi8658C::readMultipleRegs(uint8_t startReg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        i2cPort, QMI8658C_SLAVE_ADDR, &startReg, 1, data, len, pdMS_TO_TICKS(100));
}

Qmi8658C::Qmi8658C(const i2c_port_t i2CPort)
    : i2cPort(i2CPort)
{}

esp_err_t Qmi8658C::initialize()
{
    uint8_t whoami;
    if (readReg(REG_WHO_AM_I, &whoami) != ESP_OK || whoami != WHO_AM_I_VAL) {
        logCallbackService.notify("Failed to identify QMI8658C sensor");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = configure();
    if (err != ESP_OK) {
        logCallbackService.notify(
            "Failed to configure QMI8658C sensor: " + std::string(esp_err_to_name(err)));
        return err;
    }
    logCallbackService.notify("Initialized QMI8658C sensor");
    return ESP_OK;
}

esp_err_t Qmi8658C::configure()
{
    // Reset and initialize the sensor
    esp_err_t err = writeReg(REG_CTRL7, 0x00); // Disable all sensors first
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset sensor: %s", esp_err_to_name(err));
        return err;
    }

    // Configure accelerometer (±8g range, 1000Hz ODR)
    err = writeReg(REG_CTRL1, 0x63); // 0b01100011
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(err));
        return err;
    }

    // Configure gyroscope (±2000°/s range, 1000Hz ODR)
    err = writeReg(REG_CTRL2, 0x63); // 0b01100011
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(err));
        return err;
    }

    // Enable both accelerometer and gyroscope
    err = writeReg(REG_CTRL7, CTRL7_ACC_ENABLE | CTRL7_GYR_ENABLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors: %s", esp_err_to_name(err));
        return err;
    }

    // Enable data ready interrupt
    err = writeReg(REG_CTRL8, CTRL8_INT_ENABLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable data ready interrupt: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

void Qmi8658C::onSample()
{
    uint8_t data[12]; // 6 bytes for accel, 6 bytes for gyro
    if (readMultipleRegs(REG_ACC_X_L, data, sizeof(data)) != ESP_OK) {
        logCallbackService.notify("Failed to read sensor data");
        return;
    }

    // Convert raw data to sensor values
    // For demonstration, we'll just use accelerometer data
    DataPoint point{
    .accelerometer = {
    static_cast<int16_t>((data[1] << 8) | data[0]), // X
    static_cast<int16_t>((data[3] << 8) | data[2]), // Y 
    static_cast<int16_t>((data[5] << 8) | data[4])  // Z
    },
    .gyroscope = {
    static_cast<int16_t>((data[7] << 8) | data[6]), // X
    static_cast<int16_t>((data[9] << 8) | data[8]), // Y
    static_cast<int16_t>((data[11] << 8) | data[10]) // Z
    },
    .orientation{
    0,0,0}};

    const auto x = static_cast<int16_t>(
        atan2(
            point.accelerometer.y,
            sqrt(
                point.accelerometer.x * point.accelerometer.x
                + point.accelerometer.z * point.accelerometer.z))
        * 180 / M_PI); // Roll
    const auto y = static_cast<int16_t>(
        atan2(
            -point.accelerometer.x,
            point.accelerometer.z)
        * 180 / M_PI); // Pitch
    const auto z = static_cast<int16_t>(
        atan2(
            point.gyroscope.z,
            point.gyroscope.x)
        * 180 / M_PI); // Yaw

    point.orientation = {x, y, z};

    dataCallbackService.notify(point);
}

void Qmi8658C::onTerminate()
{
    // Put the sensor in sleep mode
    writeReg(REG_CTRL7, 0x00);
    logCallbackService.notify("Terminated QMI8658C sensor");
}

} // namespace Hilton