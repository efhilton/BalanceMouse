#pragma once
#include <esp_err.h>
#include <KalmanFilter.hpp>
#include <SimpleNotificationService.hpp>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Hilton {
    class Qmi8658C {
    public:
        static constexpr auto TAG{"QMI8658C"};

        // @brief  device address
        static constexpr uint8_t QMI8658_L_SLAVE_ADDRESS{0x6B};
        static constexpr uint8_t QMI8658_H_SLAVE_ADDRESS{0x6A};

        // @brief  registers default value
        static constexpr uint8_t QMI8658_REG_WHOAMI_DEFAULT{0x05};
        static constexpr uint8_t QMI8658_REG_RESET_DEFAULT{0xB0};

        //* General Purpose Registers
        static constexpr uint8_t QMI8658_REG_WHOAMI{0x00};
        static constexpr uint8_t QMI8658_REG_REVISION{0x01};

        //* Setup and Control Registers
        static constexpr uint8_t QMI8658_REG_CTRL1{0x02};
        static constexpr uint8_t QMI8658_REG_CTRL2{0x03};
        static constexpr uint8_t QMI8658_REG_CTRL3{0x04};
        static constexpr uint8_t QMI8658_REG_CTRL7{0x08};
        static constexpr uint8_t QMI8658_REG_RESET{0x60};

        // CNTRL1 Bits
        static constexpr uint8_t QMI8658_CTRL1_ADDR_AI{0x40};  // Bit 6: Address auto-increment enable

        //* Data Output Registers
        static constexpr uint8_t QMI8658_REG_AX_L{0x35};
        static constexpr uint8_t QMI8658_REG_AX_H{0x36};
        static constexpr uint8_t QMI8658_REG_AY_L{0x37};
        static constexpr uint8_t QMI8658_REG_AY_H{0x38};
        static constexpr uint8_t QMI8658_REG_AZ_L{0x39};
        static constexpr uint8_t QMI8658_REG_AZ_H{0x3A};
        static constexpr uint8_t QMI8658_REG_GX_L{0x3B};
        static constexpr uint8_t QMI8658_REG_GX_H{0x3C};
        static constexpr uint8_t QMI8658_REG_GY_L{0x3D};
        static constexpr uint8_t QMI8658_REG_GY_H{0x3E};
        static constexpr uint8_t QMI8658_REG_GZ_L{0x3F};
        static constexpr uint8_t QMI8658_REG_GZ_H{0x40};

        static constexpr uint8_t QMI8658_REG_STATUS0{0x2E};
        static constexpr uint8_t QMI8658_STATUS0_AVAIL_ACCEL{0x01};
        static constexpr uint8_t QMI8658_STATUS0_AVAIL_GYRO{0x02};

        static constexpr uint8_t QMI8658_ACCEL_EN_MASK{0x01};
        static constexpr uint8_t QMI8658_GYRO_EN_MASK{0x02};

        struct Point {
            float x;
            float y;
            float z;
        };

        struct DataPoint {
            Point rawAccel; // Linear acceleration, from sensor.
            Point rawGyro; // Angular speed, from sensor.
            Point angularAccel; // Angular acceleration, calculated.
            Point orientation; // Orientation (angle), calculated
            Point tilt; // Tilt (angle), calculated
        };

        explicit Qmi8658C(i2c_master_bus_handle_t busHandle);
        ~Qmi8658C();

        [[nodiscard]] esp_err_t initialize();
        void terminate();

        // Simple polling method - call this regularly to get sensor data
        esp_err_t readSensorData(DataPoint &point);

        // Simple notification service for data callbacks
        SimpleNotificationService<DataPoint> simpleNotificationService;

    private:
        i2c_master_bus_handle_t busHandle;
        i2c_master_dev_handle_t deviceHandle{nullptr};

        // Helper methods
        [[nodiscard]] esp_err_t writeReg(uint8_t reg, uint8_t data) const;
        esp_err_t readReg(uint8_t reg, uint8_t *data) const;
        esp_err_t readMultipleRegs(uint8_t startReg, uint8_t *data, size_t len) const;
        [[nodiscard]] esp_err_t configure() const;

        // Data processing methods
        static void populateRawAccelerometer(DataPoint &point, const uint8_t *data);
        static void populateRawGyroscope(DataPoint &point, const uint8_t *data);

        esp_err_t scanI2C() const;

        void populateAcceleration(DataPoint &point, const float &deltaTime);
        void populateOrientation(DataPoint &point, const float &deltaTime);
        void populateTilt(DataPoint &point, const float &deltaTime);

        // State variables
        Point lastOrientation{};
        Point lastGyro{};
        int64_t lastSampleTime{0};

        // Kalman filters
        KalmanFilter kalmanAccelerationX;
        KalmanFilter kalmanAccelerationY;
        KalmanFilter kalmanAccelerationZ;
        KalmanFilter kalmanTiltRoll;
        KalmanFilter kalmanTiltPitch;
        KalmanFilter kalmanOrientationX;
        KalmanFilter kalmanOrientationY;
        KalmanFilter kalmanOrientationZ;

        // Task for continuous sampling
        TaskHandle_t taskHandle{nullptr};
        bool shouldStop{false};
        static void samplingTask(void *param);
    };
} // namespace Hilton