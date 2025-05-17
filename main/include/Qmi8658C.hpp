#pragma once
#include "esp_err.h"
#include "hal/i2c_types.h"
#include <SimpleNotificationService.hpp>
#include <string>

namespace Hilton {
class Qmi8658C
{
    static constexpr auto TAG{"QMI8658C"};
    static constexpr uint8_t QMI8658C_SLAVE_ADDR = 0x6B; // I2C slave address

    // Register addresses
    static constexpr uint8_t REG_WHO_AM_I = 0x00;
    static constexpr uint8_t REG_CTRL7 = 0x07;   // Basic control
    static constexpr uint8_t REG_CTRL8 = 0x08;   // Advanced control
    static constexpr uint8_t REG_CTRL1 = 0x01;   // Accelerometer control
    static constexpr uint8_t REG_CTRL2 = 0x02;   // Gyroscope control
    static constexpr uint8_t REG_ACC_X_L = 0x35; // First data register (acc_x_l)

    // Configuration values
    static constexpr uint8_t WHO_AM_I_VAL = 0x05;
    static constexpr uint8_t CTRL7_ACC_ENABLE = 0x40;
    static constexpr uint8_t CTRL7_GYR_ENABLE = 0x80;
    static constexpr uint8_t CTRL8_INT_ENABLE = 0x01;

    i2c_port_t i2cPort;

    // Helper methods
    esp_err_t writeReg(uint8_t reg, uint8_t data);
    esp_err_t readReg(uint8_t reg, uint8_t *data);
    esp_err_t readMultipleRegs(uint8_t startReg, uint8_t *data, size_t len);

public:
    struct Point
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct DataPoint
    {
        Point accelerometer;
        Point gyroscope;
        Point orientation;
    };

    explicit Qmi8658C(i2c_port_t i2CPort);
    esp_err_t initialize();
    void onTerminate();
    void onSample();
    esp_err_t configure();
    SimpleNotificationService<DataPoint> dataCallbackService{};
    SimpleNotificationService<std::string> logCallbackService{};
};
} // namespace Hilton