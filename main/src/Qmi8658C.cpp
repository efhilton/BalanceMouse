#include <cmath>
#include <esp_log.h>
#include <esp_timer.h>
#include <Qmi8658C.hpp>
#include <driver/i2c_master.h>

namespace Hilton {
    // Constructor
    Qmi8658C::Qmi8658C(i2c_master_bus_handle_t busHandle)
        : busHandle(busHandle) {
        // Configure I2C device
        i2c_device_config_t deviceConfig = {};
        deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        deviceConfig.device_address = QMI8658_L_SLAVE_ADDRESS;
        deviceConfig.scl_speed_hz = 100000; // 100kHz

        esp_err_t err = i2c_master_bus_add_device(busHandle, &deviceConfig, &deviceHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        }
    }

    Qmi8658C::~Qmi8658C() {
        terminate();
    }

    // Simple helper to write a single register
    esp_err_t Qmi8658C::writeReg(uint8_t reg, uint8_t data) const {
        uint8_t buffer[2] = {reg, data};
        esp_err_t err = i2c_master_transmit(deviceHandle, buffer, sizeof(buffer), 1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(err));
        }
        return err;
    }

    // Simple helper to read a single register
    esp_err_t Qmi8658C::readReg(uint8_t reg, uint8_t *data) const {
        esp_err_t err = i2c_master_transmit_receive(deviceHandle, &reg, 1, data, 1, 1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(err));
        }
        return err;
    }

    // Simple helper to read multiple registers starting at a given address
    esp_err_t Qmi8658C::readMultipleRegs(uint8_t startReg, uint8_t *buffer, size_t len) const {
        esp_err_t err = i2c_master_transmit_receive(deviceHandle, &startReg, 1, buffer, len, 1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read multiple registers starting at 0x%02X: %s", startReg, esp_err_to_name(err));
        }
        return err;
    }

    // Initialize the sensor
    esp_err_t Qmi8658C::initialize() {
        // Initialize Kalman filters
        kalmanAccelerationX.initialize(1e-3, 1e-2);
        kalmanAccelerationY.initialize(1e-3, 1e-2);
        kalmanAccelerationZ.initialize(1e-3, 1e-2);
        kalmanTiltRoll.initialize(1e-3, 1e-2);
        kalmanTiltPitch.initialize(1e-3, 1e-2);
        kalmanOrientationX.initialize(1e-3, 1e-2);
        kalmanOrientationY.initialize(1e-3, 1e-2);
        kalmanOrientationZ.initialize(1e-3, 1e-2);

        // Initialize state variables
        lastGyro = {0, 0, 0};
        lastOrientation = {0, 0, 0};
        lastSampleTime = esp_timer_get_time();

        // Verify device ID
        uint8_t whoami = 0;
        if (readReg(QMI8658_REG_WHOAMI, &whoami) != ESP_OK || whoami != QMI8658_REG_WHOAMI_DEFAULT) {
            ESP_LOGE(TAG, "WHOAMI failed or unexpected: 0x%02X", whoami);
            return ESP_FAIL;
        }

        // Reset sensor
        esp_err_t err = writeReg(QMI8658_REG_RESET, QMI8658_REG_RESET_DEFAULT);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reset sensor: %s", esp_err_to_name(err));
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset

        // Configure sensor
        err = configure();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure sensor: %s", esp_err_to_name(err));
            return err;
        }

        // Start sampling task
        shouldStop = false;
        const auto created = xTaskCreate(&Qmi8658C::samplingTask, "QMI8658C-Sample", 4096, this, 5, &taskHandle);
        if (created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create sampling task");
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "QMI8658C initialized successfully");
        return ESP_OK;
    }

    // Configure sensor registers
    esp_err_t Qmi8658C::configure() const {
        esp_err_t err;

        // Disable sensors first
        err = writeReg(QMI8658_REG_CTRL7, 0x00);
        if (err != ESP_OK) return err;

        // Configure accelerometer: ±2G range, 50Hz ODR
        err = writeReg(QMI8658_REG_CTRL2, 0x05);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure accelerometer");
            return err;
        }

        // Configure gyroscope: ±512DPS range, 50Hz ODR
        err = writeReg(QMI8658_REG_CTRL3, 0x55);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure gyroscope");
            return err;
        }

        // Enable both accelerometer and gyroscope
        err = writeReg(QMI8658_REG_CTRL7, QMI8658_ACCEL_EN_MASK | QMI8658_GYRO_EN_MASK);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable sensors");
            return err;
        }

        // Wait for sensors to stabilize
        vTaskDelay(pdMS_TO_TICKS(50));

        ESP_LOGI(TAG, "QMI8658C configured successfully");
        return ESP_OK;
    }

    // Simple polling method to read sensor data
    esp_err_t Qmi8658C::readSensorData(DataPoint &point) {
        uint8_t data[12]; // 6 bytes for accel, 6 bytes for gyro

        // Read all sensor data in one go
        esp_err_t err = readMultipleRegs(QMI8658_REG_AX_L, data, sizeof(data));
        if (err != ESP_OK) {
            return err;
        }

        // Calculate time delta
        const int64_t currentTime = esp_timer_get_time();
        const float deltaTime = static_cast<float>(currentTime - lastSampleTime) / 1000000.0f;
        lastSampleTime = currentTime;

        // Process raw data
        populateRawAccelerometer(point, data);
        populateRawGyroscope(point, data);
        populateAcceleration(point, deltaTime);
        populateOrientation(point, deltaTime);
        populateTilt(point, deltaTime);

        return ESP_OK;
    }

    // Sampling task that runs continuously
    void Qmi8658C::samplingTask(void *param) {
        auto *self = static_cast<Qmi8658C *>(param);
        DataPoint data{};
        constexpr TickType_t SAMPLE_RATE_MS = pdMS_TO_TICKS(20); // 50Hz sampling

        ESP_LOGI(TAG, "Sampling task started");

        while (!self->shouldStop) {
            if (self->readSensorData(data) == ESP_OK) {
                // Notify callbacks
                self->dataService.notify(data);
            } else {
                ESP_LOGW(TAG, "Failed to read sensor data");
            }

            vTaskDelay(SAMPLE_RATE_MS);
        }

        ESP_LOGI(TAG, "Sampling task stopped");
        vTaskDelete(nullptr);
    }

    void Qmi8658C::terminate() {
        if (taskHandle != nullptr) {
            shouldStop = true;
            // Wait for task to finish
            while (eTaskGetState(taskHandle) != eDeleted) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            taskHandle = nullptr;
        }

        // Disable sensors
        if (deviceHandle != nullptr) {
            if (writeReg(QMI8658_REG_CTRL7, 0x00) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to disable sensors");
            }
        }

        ESP_LOGI(TAG, "QMI8658C terminated");
    }

    void Qmi8658C::populateAcceleration(DataPoint &point, const float &deltaTime) {
        // Calculate angular acceleration as the rate of change of angular velocity
        point.acceleration.x = kalmanAccelerationX.estimate((point.gyroscope.x - lastGyro.x) / deltaTime, 0, deltaTime);
        point.acceleration.y = kalmanAccelerationY.estimate((point.gyroscope.y - lastGyro.y) / deltaTime, 0, deltaTime);
        point.acceleration.z = kalmanAccelerationZ.estimate((point.gyroscope.z - lastGyro.z) / deltaTime, 0, deltaTime);

        lastGyro = point.gyroscope;
    }

    void Qmi8658C::populateTilt(DataPoint &point, const float &deltaTime) {
        constexpr float g = 9.81f;
        const float ax = point.accelerometer.x / g;
        const float ay = point.accelerometer.y / g;
        const float az = point.accelerometer.z / g;

        // Direct tilt calculation from accelerometer
        const float roll = std::atan2(ay, az);
        const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        // Use Kalman filters to smooth raw tilt values
        point.tilt.x = kalmanTiltRoll.estimate(roll * 180.0f / M_PI, 0, 0.0f);
        point.tilt.y = kalmanTiltPitch.estimate(pitch * 180.0f / M_PI, 0, 0.0f);
        point.tilt.z = 0; // Yaw cannot be determined from accelerometer alone
    }

    void Qmi8658C::populateOrientation(DataPoint &point, const float &deltaTime) {
        constexpr float ALPHA = 0.98f; // Complementary filter weight

        // Integrate gyro data to estimate angular changes
        lastOrientation.x += point.gyroscope.x * deltaTime; // Roll
        lastOrientation.y += point.gyroscope.y * deltaTime; // Pitch
        lastOrientation.z += point.gyroscope.z * deltaTime; // Yaw

        // Calculate pitch and roll from accelerometer
        const float accelPitch = std::atan2(point.accelerometer.x,
                                            std::sqrt(point.accelerometer.y * point.accelerometer.y +
                                                      point.accelerometer.z * point.accelerometer.z));
        const float accelRoll = std::atan2(point.accelerometer.y,
                                           std::sqrt(point.accelerometer.x * point.accelerometer.x +
                                                     point.accelerometer.z * point.accelerometer.z));

        // Use complementary filter to fuse gyro and accelerometer
        lastOrientation.x = ALPHA * lastOrientation.x + (1 - ALPHA) * (accelRoll * 180.0f / M_PI);
        lastOrientation.y = ALPHA * lastOrientation.y + (1 - ALPHA) * (accelPitch * 180.0f / M_PI);

        // Apply Kalman filters
        point.orientation.x = kalmanOrientationX.estimate(lastOrientation.x, point.gyroscope.x, deltaTime);
        point.orientation.y = kalmanOrientationY.estimate(lastOrientation.y, point.gyroscope.y, deltaTime);
        point.orientation.z = kalmanOrientationZ.estimate(lastOrientation.z, point.gyroscope.z, deltaTime);
    }

    void Qmi8658C::populateRawAccelerometer(DataPoint &point, const uint8_t *data) {
        constexpr float ACCEL_SCALE = 2.0f * 9.81 / 32768.0f; // For ±2G range, in m/s^2
        point.accelerometer.x = static_cast<float>(static_cast<int16_t>((data[1] << 8) | data[0])) * ACCEL_SCALE;
        point.accelerometer.y = static_cast<float>(static_cast<int16_t>((data[3] << 8) | data[2])) * ACCEL_SCALE;
        point.accelerometer.z = static_cast<float>(static_cast<int16_t>((data[5] << 8) | data[4])) * ACCEL_SCALE;
    }

    void Qmi8658C::populateRawGyroscope(DataPoint &point, const uint8_t *data) {
        constexpr float GYRO_SCALE = 512.0f / 32768.0f; // For ±512DPS range
        point.gyroscope.x = static_cast<float>(static_cast<int16_t>((data[7] << 8) | data[6])) * GYRO_SCALE;
        point.gyroscope.y = static_cast<float>(static_cast<int16_t>((data[9] << 8) | data[8])) * GYRO_SCALE;
        point.gyroscope.z = static_cast<float>(static_cast<int16_t>((data[11] << 8) | data[10])) * GYRO_SCALE;
    }
} // namespace Hilton
