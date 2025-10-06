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

            // Debug: Print what we actually received on failure
            ESP_LOGE(TAG, "Received bytes on error:");
            for (size_t i = 0; i < len && i < 12; i++) {
                ESP_LOGE(TAG, "  [%zu] = 0x%02X", i, buffer[i]);
            }
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

        ESP_LOGI(TAG, "Scanning I2C bus for devices...");
        if (scanI2C() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to scan devices");
            return ESP_FAIL;
        }

        // Verify device ID MULTIPLE times to ensure I2C is working
        for (int attempt = 0; attempt < 3; attempt++) {
            uint8_t whoami = 0;
            esp_err_t err = readReg(QMI8658_REG_WHOAMI, &whoami);
            ESP_LOGI(TAG, "WHO_AM_I attempt %d: err=%s, value=0x%02X", attempt + 1, esp_err_to_name(err), whoami);

            if (err == ESP_OK && whoami == QMI8658_REG_WHOAMI_DEFAULT) {
                ESP_LOGI(TAG, "Device ID verified successfully");
                break;
            }

            if (attempt == 2) {
                ESP_LOGE(TAG, "Device ID verification failed after 3 attempts");
                return ESP_FAIL;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Perform a more thorough reset
        ESP_LOGI(TAG, "Performing sensor reset...");

        // First, try to reset via register
        esp_err_t err = writeReg(QMI8658_REG_RESET, QMI8658_REG_RESET_DEFAULT);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write reset register: %s", esp_err_to_name(err));
            return err;
        }

        // Wait longer for reset to complete
        vTaskDelay(pdMS_TO_TICKS(200)); // Increased from 100ms

        // Verify device comes back online
        uint8_t whoami = 0;
        for (int retry = 0; retry < 10; retry++) {
            err = readReg(QMI8658_REG_WHOAMI, &whoami);
            ESP_LOGI(TAG, "Post-reset WHO_AM_I attempt %d: err=%s, value=0x%02X",
                     retry + 1, esp_err_to_name(err), whoami);

            if (err == ESP_OK && whoami == QMI8658_REG_WHOAMI_DEFAULT) {
                ESP_LOGI(TAG, "Device reset successful");
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (err != ESP_OK || whoami != QMI8658_REG_WHOAMI_DEFAULT) {
            ESP_LOGE(TAG, "Device reset failed - WHO_AM_I: expected 0x%02X, got 0x%02X",
                     QMI8658_REG_WHOAMI_DEFAULT, whoami);
            return ESP_FAIL;
        }

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

        // Enable ONLY address auto-increment (bit 6)
        err = writeReg(QMI8658_REG_CTRL1, QMI8658_CTRL1_ADDR_AI);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable address auto-increment");
            return err;
        }
        ESP_LOGI(TAG, "Enabled address auto-increment for multi-byte reads");

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

    esp_err_t Qmi8658C::readSensorData(DataPoint &point) {
        // Check status register to see if data is ready
        uint8_t status = 0;
        esp_err_t err = readReg(QMI8658_REG_STATUS0, &status);
        if (err == ESP_OK) {
            ESP_LOGD(TAG, "Status register: 0x%02X (Accel ready: %s, Gyro ready: %s)",
                     status,
                     (status & QMI8658_STATUS0_AVAIL_ACCEL) ? "YES" : "NO",
                     (status & QMI8658_STATUS0_AVAIL_GYRO) ? "YES" : "NO");
        }

        uint8_t data[12]; // 6 bytes for accel, 6 bytes for gyro

        // Read all sensor data in one go
        err = readMultipleRegs(QMI8658_REG_AX_L, data, sizeof(data));
        if (err != ESP_OK) {
            return err;
        }

        // Debug: Check for the problematic 0x84 pattern
        bool allSame84 = true;
        for (int i = 0; i < 11; i++) {
            // Check first 11 bytes
            if (data[i] != 0x84) {
                allSame84 = false;
                break;
            }
        }
        if (allSame84) {
            ESP_LOGW(TAG, "⚠ Detected 0x84 pattern - sensor not providing real data");
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
                self->simpleNotificationService.notify(data);
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
        point.acceleration.x = kalmanAccelerationX.estimate((point.rawGyro.x - lastGyro.x) / deltaTime, 0, deltaTime);
        point.acceleration.y = kalmanAccelerationY.estimate((point.rawGyro.y - lastGyro.y) / deltaTime, 0, deltaTime);
        point.acceleration.z = kalmanAccelerationZ.estimate((point.rawGyro.z - lastGyro.z) / deltaTime, 0, deltaTime);

        lastGyro = point.rawGyro;
    }

    void Qmi8658C::populateTilt(DataPoint &point, const float &deltaTime) {
        constexpr float g = 9.81f;
        const float ax = point.rawAccel.x / g;
        const float ay = point.rawAccel.y / g;
        const float az = point.rawAccel.z / g;

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
        lastOrientation.x += point.rawGyro.x * deltaTime; // Roll
        lastOrientation.y += point.rawGyro.y * deltaTime; // Pitch
        lastOrientation.z += point.rawGyro.z * deltaTime; // Yaw

        // Calculate pitch and roll from accelerometer
        const float accelPitch = std::atan2(point.rawAccel.x,
                                            std::sqrt(point.rawAccel.y * point.rawAccel.y +
                                                      point.rawAccel.z * point.rawAccel.z));
        const float accelRoll = std::atan2(point.rawAccel.y,
                                           std::sqrt(point.rawAccel.x * point.rawAccel.x +
                                                     point.rawAccel.z * point.rawAccel.z));

        // Use complementary filter to fuse gyro and accelerometer
        lastOrientation.x = ALPHA * lastOrientation.x + (1 - ALPHA) * (accelRoll * 180.0f / M_PI);
        lastOrientation.y = ALPHA * lastOrientation.y + (1 - ALPHA) * (accelPitch * 180.0f / M_PI);

        // Apply Kalman filters
        point.orientation.x = kalmanOrientationX.estimate(lastOrientation.x, point.rawGyro.x, deltaTime);
        point.orientation.y = kalmanOrientationY.estimate(lastOrientation.y, point.rawGyro.y, deltaTime);
        point.orientation.z = kalmanOrientationZ.estimate(lastOrientation.z, point.rawGyro.z, deltaTime);
    }

    void Qmi8658C::populateRawAccelerometer(DataPoint &point, const uint8_t *data) {
        constexpr float ACCEL_SCALE = 2.0f * 9.81 / 32768.0f; // For ±2G range, in m/s^2

        // Read raw values
        float rawX = static_cast<float>(static_cast<int16_t>((data[1] << 8) | data[0])) * ACCEL_SCALE;
        float rawY = static_cast<float>(static_cast<int16_t>((data[3] << 8) | data[2])) * ACCEL_SCALE;
        float rawZ = static_cast<float>(static_cast<int16_t>((data[5] << 8) | data[4])) * ACCEL_SCALE;

        // Accelerometers point in the direction of the force that OPPOSES gravity.
        // Let's flip them all so that it gives a positive 9.8 m/s^2 in the direction
        // that the sensor is pointing down. So, if x is down, we get +9.8. If y points down, we get +9.8.
        // and if z points down, we get a positive 9.8.
        point.rawAccel.x = -rawX; // Flip X
        point.rawAccel.y = -rawY; // Flip Y
        point.rawAccel.z = -rawZ; // Flip Z
    }

    void Qmi8658C::populateRawGyroscope(DataPoint &point, const uint8_t *data) {
        constexpr float GYRO_SCALE = 512.0f / 32768.0f; // For ±512DPS range
        point.rawGyro.x = static_cast<float>(static_cast<int16_t>((data[7] << 8) | data[6])) * GYRO_SCALE;
        point.rawGyro.y = static_cast<float>(static_cast<int16_t>((data[9] << 8) | data[8])) * GYRO_SCALE;
        point.rawGyro.z = static_cast<float>(static_cast<int16_t>((data[11] << 8) | data[10])) * GYRO_SCALE;
    }

    esp_err_t Qmi8658C::scanI2C() const {
        ESP_LOGI(TAG, "=== I2C DEVICE COMMUNICATION TEST ===");

        // Test communication with our configured device (0x6B)
        ESP_LOGI(TAG, "Testing communication with QMI8658C at address 0x%02X...", QMI8658_L_SLAVE_ADDRESS);

        // Test 1: WHO_AM_I register
        uint8_t whoami = 0xFF; // Initialize with invalid value
        esp_err_t err = readReg(QMI8658_REG_WHOAMI, &whoami);

        ESP_LOGI(TAG, "WHO_AM_I test: err=%s, value=0x%02X (expected=0x%02X)",
                 esp_err_to_name(err), whoami, QMI8658_REG_WHOAMI_DEFAULT);

        if (err == ESP_OK) {
            if (whoami == QMI8658_REG_WHOAMI_DEFAULT) {
                ESP_LOGI(TAG, "✓ Device responds correctly - I2C communication OK");
            } else {
                ESP_LOGW(TAG, "⚠ Device responds but with unexpected WHO_AM_I value");
            }
        } else {
            ESP_LOGE(TAG, "✗ Failed to read WHO_AM_I - I2C communication problem");
            return err;
        }

        // Test 2: REVISION register (should be readable)
        uint8_t revision = 0xFF;
        err = readReg(QMI8658_REG_REVISION, &revision);
        ESP_LOGI(TAG, "REVISION test: err=%s, value=0x%02X", esp_err_to_name(err), revision);

        // Test 3: Try reading all control registers to see current state
        uint8_t ctrl2 = 0xFF, ctrl3 = 0xFF, ctrl7 = 0xFF;
        esp_err_t err2 = readReg(QMI8658_REG_CTRL2, &ctrl2);
        esp_err_t err3 = readReg(QMI8658_REG_CTRL3, &ctrl3);
        esp_err_t err7 = readReg(QMI8658_REG_CTRL7, &ctrl7);

        ESP_LOGI(TAG, "Control registers:");
        ESP_LOGI(TAG, "  CTRL2 (Accel): err=%s, value=0x%02X", esp_err_to_name(err2), ctrl2);
        ESP_LOGI(TAG, "  CTRL3 (Gyro):  err=%s, value=0x%02X", esp_err_to_name(err3), ctrl3);
        ESP_LOGI(TAG, "  CTRL7 (Enable): err=%s, value=0x%02X", esp_err_to_name(err7), ctrl7);

        // Test 4: Status register
        uint8_t status = 0xFF;
        esp_err_t errStatus = readReg(QMI8658_REG_STATUS0, &status);
        ESP_LOGI(TAG, "STATUS0: err=%s, value=0x%02X", esp_err_to_name(errStatus), status);
        if (errStatus == ESP_OK) {
            ESP_LOGI(TAG, "  Accel data ready: %s", (status & QMI8658_STATUS0_AVAIL_ACCEL) ? "YES" : "NO");
            ESP_LOGI(TAG, "  Gyro data ready:  %s", (status & QMI8658_STATUS0_AVAIL_GYRO) ? "YES" : "NO");
        }

        // Test 5: Sample a few data registers to see if they're changing
        ESP_LOGI(TAG, "Sampling data registers (3 samples with 100ms intervals):");
        for (int sample = 0; sample < 3; sample++) {
            uint8_t ax_l, ax_h, gx_l, gx_h;
            readReg(QMI8658_REG_AX_L, &ax_l);
            readReg(QMI8658_REG_AX_H, &ax_h);
            readReg(QMI8658_REG_GX_L, &gx_l);
            readReg(QMI8658_REG_GX_H, &gx_h);

            ESP_LOGI(TAG, "  Sample %d: AX_L=0x%02X AX_H=0x%02X GX_L=0x%02X GX_H=0x%02X",
                     sample + 1, ax_l, ax_h, gx_l, gx_h);

            if (sample < 2) {
                // Don't delay after last sample
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        // Test 6: Read multiple registers at once (this is what fails in your case)
        ESP_LOGI(TAG, "Testing multi-register read (12 bytes from 0x%02X):", QMI8658_REG_AX_L);
        uint8_t multiData[12];
        esp_err_t errMulti = readMultipleRegs(QMI8658_REG_AX_L, multiData, sizeof(multiData));
        ESP_LOGI(TAG, "Multi-read result: %s", esp_err_to_name(errMulti));

        if (errMulti == ESP_OK) {
            ESP_LOGI(TAG, "Multi-read data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                     multiData[0], multiData[1], multiData[2], multiData[3],
                     multiData[4], multiData[5], multiData[6], multiData[7],
                     multiData[8], multiData[9], multiData[10], multiData[11]);

            // Check for problematic patterns
            bool allSame = true;
            for (int i = 1; i < 12; i++) {
                if (multiData[i] != multiData[0]) {
                    allSame = false;
                    break;
                }
            }

            if (allSame) {
                ESP_LOGW(TAG, "⚠ WARNING: All bytes identical (0x%02X) - sensor may not be sampling", multiData[0]);
            } else {
                ESP_LOGI(TAG, "✓ Multi-read data shows variation - looks good");
            }
        }

        ESP_LOGI(TAG, "=== I2C TEST COMPLETE ===");
        return ESP_OK;
    }
} // namespace Hilton
