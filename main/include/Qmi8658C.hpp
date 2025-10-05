#pragma once
#include <esp_err.h>
#include <KalmanFilter.hpp>
#include <QueuedNotificationService.hpp>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <freertos/event_groups.h>
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
        static constexpr uint8_t QMI8658_REG_STATUS_DEFAULT{0x03};
        static constexpr uint8_t QMI8658_REG_RESET_DEFAULT{0xB0};

        //* General Purpose Registers
        static constexpr uint8_t QMI8658_REG_WHOAMI{0x00};
        static constexpr uint8_t QMI8658_REG_REVISION{0x01};

        //* Setup and Control Registers
        static constexpr uint8_t QMI8658_REG_CTRL1{0x02};
        static constexpr uint8_t QMI8658_REG_CTRL2{0x03};
        static constexpr uint8_t QMI8658_REG_CTRL3{0x04};
        // Reserved                                     {0x05};
        static constexpr uint8_t QMI8658_REG_CTRL5{0x06};
        // Reserved                                     {0x07};
        static constexpr uint8_t QMI8658_REG_CTRL7{0x08};
        static constexpr uint8_t QMI8658_REG_CTRL8{0x09};
        static constexpr uint8_t QMI8658_REG_CTRL9{0x0A};

        //* Host Controlled Calibration Registers (See CTRL9, Usage is Optional};
        static constexpr uint8_t QMI8658_REG_CAL1_L{0x0B};
        static constexpr uint8_t QMI8658_REG_CAL1_H{0x0C};
        static constexpr uint8_t QMI8658_REG_CAL2_L{0x0D};
        static constexpr uint8_t QMI8658_REG_CAL2_H{0x0E};
        static constexpr uint8_t QMI8658_REG_CAL3_L{0x0F};
        static constexpr uint8_t QMI8658_REG_CAL3_H{0x10};
        static constexpr uint8_t QMI8658_REG_CAL4_L{0x11};
        static constexpr uint8_t QMI8658_REG_CAL4_H{0x12};

        //* FIFO Registers
        static constexpr uint8_t QMI8658_REG_FIFO_WTM_TH{0x13};
        static constexpr uint8_t QMI8658_REG_FIFO_CTRL{0x14};
        static constexpr uint8_t QMI8658_REG_FIFO_COUNT{0x15};
        static constexpr uint8_t QMI8658_REG_FIFO_STATUS{0x16};
        static constexpr uint8_t QMI8658_REG_FIFO_DATA{0x17};

        //* Status Registers
        static constexpr uint8_t QMI8658_REG_STATUS_INT{0x2D};
        static constexpr uint8_t QMI8658_REG_STATUS0{0x2E};
        static constexpr uint8_t QMI8658_REG_STATUS1{0x2F};

        //* Timestamp Register
        static constexpr uint8_t QMI8658_REG_TIMESTAMP_L{0x30};
        static constexpr uint8_t QMI8658_REG_TIMESTAMP_M{0x31};
        static constexpr uint8_t QMI8658_REG_TIMESTAMP_H{0x32};

        //* Data Output Registers (16 bits 2’s Complement Except COD Sensor Data};
        static constexpr uint8_t QMI8658_REG_TEMPERATURE_L{0x33};
        static constexpr uint8_t QMI8658_REG_TEMPERATURE_H{0x34};
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

        //* COD Indication and General Purpose Registers

        // Calibration-On-Demand status register
        static constexpr uint8_t QMI8658_REG_COD_STATUS{0x46};
        static constexpr uint8_t QMI8658_REG_DQW_L{0x49};
        static constexpr uint8_t QMI8658_REG_DQW_H{0x4A};
        static constexpr uint8_t QMI8658_REG_DQX_L{0x4B};
        static constexpr uint8_t QMI8658_REG_DQX_H{0x4C};

        static constexpr uint8_t QMI8658_REG_DQY_L{0x4D};
        static constexpr uint8_t QMI8658_REG_DQY_H{0x4E};
        static constexpr uint8_t QMI8658_REG_DQZ_L{0x4F};
        static constexpr uint8_t QMI8658_REG_DQZ_H{0x50};

        static constexpr uint8_t QMI8658_REG_DVX_L{0x51};
        static constexpr uint8_t QMI8658_REG_DVX_H{0x52};
        static constexpr uint8_t QMI8658_REG_DVY_L{0x53};
        static constexpr uint8_t QMI8658_REG_DVY_H{0x54};
        static constexpr uint8_t QMI8658_REG_DVZ_L{0x55};
        static constexpr uint8_t QMI8658_REG_DVZ_H{0x56};

        //* Activity Detection Output Registers
        static constexpr uint8_t QMI8658_REG_TAP_STATUS{0x59};
        static constexpr uint8_t QMI8658_REG_STEP_CNT_LOW{0x5A};
        static constexpr uint8_t QMI8658_REG_STEP_CNT_MID{0x5B};
        static constexpr uint8_t QMI8658_REG_STEP_CNT_HIGH{0x5C};
        static constexpr uint8_t QMI8658_REG_RESET{0x60};

        //* Reset Register
        static constexpr uint8_t QMI8658_REG_RST_RESULT{0x4D};
        static constexpr uint8_t QMI8658_REG_RST_RESULT_VAL{0x80};

        static constexpr uint8_t STATUS0_ACCEL_AVAIL{0x01};
        static constexpr uint8_t STATUS0_GYRO_AVAIL{0x02};
        static constexpr uint8_t QMI8658_ACCEL_LPF_MASK{0xF9};
        static constexpr uint8_t QMI8658_GYRO_LPF_MASK{0x9F};

        static constexpr uint8_t QMI8658_ACCEL_EN_MASK{0x01};
        static constexpr uint8_t QMI8658_GYRO_EN_MASK{0x02};
        static constexpr uint8_t QMI8658_SYNC_SIMPL_EN_MASK{0x80};
        static constexpr uint8_t QMI8658_ACCEL_GYRO_EN_MASK{
            QMI8658_ACCEL_EN_MASK |
            QMI8658_GYRO_EN_MASK
        };

        static constexpr uint8_t QMI8658_FIFO_MAP_INT1{0x04}; // ctrl1

        struct Point {
            float x;
            float y;
            float z;
        };

        struct DataPoint {
            Point accelerometer; // Linear acceleration, from sensor.
            Point gyroscope; // Angular speed, from sensor.
            Point acceleration; // Angular acceleration, calculated.
            Point orientation; // Orientation (angle), calculated
            Point tilt; // Tilt (angle), calculated
        };

    private:
        i2c_master_bus_handle_t busHandle;
        i2c_master_dev_handle_t deviceHandle{nullptr};

        // Helper methods
        [[nodiscard]] esp_err_t writeReg(uint8_t reg, uint8_t data) const;

        esp_err_t readReg(uint8_t reg, uint8_t *data) const;

        esp_err_t readMultipleRegs(uint8_t startReg, uint8_t *data, size_t len) const;

    public:
        enum class QueueKey { DATA_READY };

        enum EventBits : EventBits_t {
            DO_STOP = BIT0,
            DATA_IS_AVAILABLE = BIT1,
            IS_RUNNING = BIT2,
            IS_STOPPED = BIT3,
        };

        explicit Qmi8658C(i2c_master_bus_handle_t busHandle);

        [[nodiscard]] esp_err_t initialize(gpio_num_t int1, gpio_num_t int2);
        void terminate();
        [[nodiscard]] esp_err_t disableAllSensors() const;
        static void populateRawAccelerometer(DataPoint &point, const uint8_t *data);
        static void populateRawGyroscope(DataPoint &point, const uint8_t *data);
        void onTerminate() const;
        void populateAcceleration(DataPoint &point, const float &deltaTime);
        void populateOrientation(DataPoint &point, const float &deltaTime);
        void populateTilt(DataPoint &point, const float &deltaTime);
        esp_err_t onSample(DataPoint &point);
        [[nodiscard]] esp_err_t configure() const;

        // Public data notification service (no internal consumer task)
        QueuedNotificationService<QueueKey, DataPoint> dataService{
            16, "QMI8658C-DataQ", 2048, tskIDLE_PRIORITY + 2, false
        };

        Point lastOrientation{};
        Point lastGyro{};
        // Kalman filters for acceleration
        KalmanFilter kalmanAccelerationX; // Smoothing X-axis acceleration
        KalmanFilter kalmanAccelerationY; // Smoothing Y-axis acceleration
        KalmanFilter kalmanAccelerationZ; // Smoothing Z-axis acceleration

        // Kalman filters for tilt
        KalmanFilter kalmanTiltRoll; // Smoothing roll tilt
        KalmanFilter kalmanTiltPitch; // Smoothing pitch tilt

        // Kalman filters for orientation
        KalmanFilter kalmanOrientationX; // Smoothing X-axis orientation (roll)
        KalmanFilter kalmanOrientationY; // Smoothing Y-axis orientation (pitch)
        KalmanFilter kalmanOrientationZ; // Smoothing Z-axis orientation (yaw)

    private:
        static void IRAM_ATTR gpioIsrHandler(void *arg);

        static void workerTask(void *param);

        EventGroupHandle_t eventGroup{nullptr};
        TaskHandle_t taskHandle{nullptr};
        gpio_num_t int1Pin{GPIO_NUM_NC};
        gpio_num_t int2Pin{GPIO_NUM_NC};
    };
} // namespace Hilton
