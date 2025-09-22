#include "Qmi8658C.hpp"
#include "esp_log.h"

#include <cmath>
#include <cstring>
#include <driver/i2c.h>
#include <esp_timer.h>

namespace Hilton {

constexpr const char *TAG = "QMI8658C";

// Constructor
Qmi8658C::Qmi8658C(const i2c_port_t i2CPort)
    : i2cPort(i2CPort)
{}

// Simplified helper to write a single register
esp_err_t Qmi8658C::writeReg(uint8_t reg, uint8_t data) const
{
    uint8_t buffer[2] = {reg, data};
    esp_err_t err = i2c_master_write_to_device(i2cPort, QMI8658_L_SLAVE_ADDRESS, buffer, sizeof(buffer), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(err));
    }
    return err;
}

// Simplified helper to read a single register
esp_err_t Qmi8658C::readReg(uint8_t reg, uint8_t *data) const
{
    esp_err_t err = i2c_master_write_read_device(i2cPort, QMI8658_L_SLAVE_ADDRESS, &reg, 1, data, 1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(err));
    }
    return err;
}

// Simplified helper to read multiple registers starting at a given address
esp_err_t Qmi8658C::readMultipleRegs(uint8_t startReg, uint8_t *buffer, size_t len) const
{
    esp_err_t err = i2c_master_write_read_device(i2cPort, QMI8658_L_SLAVE_ADDRESS, &startReg, 1, buffer, len, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read multiple registers starting at 0x%02X: %s", startReg, esp_err_to_name(err));
    }
    return err;
}

// Initialize the task, GPIOs, and sensor context
esp_err_t Qmi8658C::initialize(gpio_num_t int1, gpio_num_t int2)
{
    // Initialize Kalman filters for acceleration
    kalmanAccelerationX.initialize(1e-3, 1e-2);
    kalmanAccelerationY.initialize(1e-3, 1e-2);
    kalmanAccelerationZ.initialize(1e-3, 1e-2);

    // Initialize Kalman filters for tilt
    kalmanTiltRoll.initialize(1e-3, 1e-2);
    kalmanTiltPitch.initialize(1e-3, 1e-2);

    // Initialize Kalman filters for lastOrientation
    kalmanOrientationX.initialize(1e-3, 1e-2);
    kalmanOrientationY.initialize(1e-3, 1e-2);
    kalmanOrientationZ.initialize(1e-3, 1e-2);

    // Initialize last gyroscope readings to zero
    lastGyro = {0, 0, 0};

    // Save pins
    int1Pin = int1;
    int2Pin = int2;

    // Create event group
    if (eventGroup == nullptr) {
        eventGroup = xEventGroupCreate();
    }
    if (eventGroup == nullptr) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    // Configure GPIOs
    gpio_reset_pin(int1Pin);
    gpio_reset_pin(int2Pin);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << int1Pin) | (1ULL << int2Pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    esp_err_t gret = gpio_config(&io_conf);
    if (gret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT pins: %s", esp_err_to_name(gret));
        return gret;
    }
    gpio_set_intr_type(int1Pin, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(int2Pin, GPIO_INTR_ANYEDGE);

    // Install ISR service and add handlers
    gpio_install_isr_service(0);
    gpio_isr_handler_add(int1Pin, &Qmi8658C::gpioIsrHandler, this);
    gpio_isr_handler_add(int2Pin, &Qmi8658C::gpioIsrHandler, this);
    gpio_intr_enable(int1Pin);
    gpio_intr_enable(int2Pin);

    // Start task
    auto created = xTaskCreate(&Qmi8658C::taskEntry, "QMI8658C-Task", 4096, this, 5, &taskHandle);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create QMI8658C task");
        return ESP_FAIL;
    }

    // Wait for task to report running
    EventBits_t bits = xEventGroupWaitBits(eventGroup, IS_RUNNING, pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
    if ((bits & IS_RUNNING) == 0) {
        ESP_LOGE(TAG, "QMI8658C task failed to start");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void IRAM_ATTR Qmi8658C::gpioIsrHandler(void *arg)
{
    auto *self = static_cast<Qmi8658C *>(arg);
    BaseType_t hpw = pdFALSE;
    xEventGroupSetBitsFromISR(self->eventGroup, DATA_IS_AVAILABLE, &hpw);
    if (hpw) {
        portYIELD_FROM_ISR();
    }
}

void Qmi8658C::taskEntry(void *param)
{
    auto *self = static_cast<Qmi8658C *>(param);

    // Configure sensor on task startup
    // Verify device, reset, then configure
    uint8_t whoami = 0;
    if (self->readReg(QMI8658_REG_WHOAMI, &whoami) != ESP_OK || whoami != QMI8658_REG_WHOAMI_DEFAULT) {
        ESP_LOGE(TAG, "WHOAMI failed or unexpected: 0x%02X", whoami);
    } else {
        self->writeReg(QMI8658_REG_RESET, QMI8658_REG_RESET_DEFAULT);
        vTaskDelay(pdMS_TO_TICKS(100));
        if (self->configure() != ESP_OK) {
            ESP_LOGE(TAG, "configure() failed in task startup");
        }
    }

    // Signal running
    xEventGroupSetBits(self->eventGroup, IS_RUNNING);

    DataPoint data{};

    while (true) {
        // Wait for interrupt or stop signal
        EventBits_t bits = xEventGroupWaitBits(self->eventGroup, DATA_IS_AVAILABLE | DO_STOP, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & DO_STOP) {
            break;
        }

        if (bits & DATA_IS_AVAILABLE) {
            // Read interrupt status to clear it
            uint8_t status = 0;
            self->readReg(QMI8658_REG_STATUS_INT, &status);

            // Fetch sensor data
            if (self->onSample(data) == ESP_OK) {
                // Notify via data service
                self->dataService.notify(QueueKey::DataReady, data);
                ESP_LOGI(TAG, "Sensor data processed");
            } else {
                ESP_LOGE(TAG, "Failed to process sensor data");
            }
        }
    }

    // Shutdown sequence
    self->onTerminate();
    xEventGroupSetBits(self->eventGroup, IS_STOPPED);
    vTaskDelete(nullptr);
}

// Disable all sensors
esp_err_t Qmi8658C::disableAllSensors() const
{
    return writeReg(QMI8658_REG_CTRL7,
                    0x00); // Disable both accelerometer and gyroscope
}

// Sensor interrupt and initialization configuration
esp_err_t Qmi8658C::configure() const
{
    esp_err_t err = disableAllSensors(); // Disable sensors before configuration
    if (err != ESP_OK) {
        return err;
    }

    // Configure accelerometer: ±2G range, 58.75Hz ODR
    constexpr uint8_t accelConfig = 0b00000111;
    err = writeReg(QMI8658_REG_CTRL2, accelConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "CTRL2 (Accelerometer): 0x%02X", accelConfig);

    // Configure gyroscope: ±128DPS range, 29.375Hz ODR
    constexpr uint8_t gyroConfig = 0b00110111;
    err = writeReg(QMI8658_REG_CTRL3, gyroConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "CTRL3 (Gyroscope): 0x%02X", gyroConfig);

    // Enable accelerometer and gyroscope
    constexpr uint8_t ctrl7Reg = QMI8658_ACCEL_EN_MASK | QMI8658_GYRO_EN_MASK;
    err = writeReg(QMI8658_REG_CTRL7, ctrl7Reg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable accelerometer/gyroscope sensors: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "CTRL7 (Sensor Enable): 0x%02X", ctrl7Reg);

    // Route DRDY interrupt to INT1 and INT2 as active-low, push-pull
    // 0x48: Active-low, push-pull configuration
    // 0x02: Data-ready routed to INT1
    // 0x08: Data-ready routed to INT2
    constexpr uint8_t interruptConfig = 0b01001010;
    err = writeReg(QMI8658_REG_CTRL8, interruptConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure interrupts: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "CTRL8 (Interrupts): 0x%02X", interruptConfig);

    // Optionally, verify STATUS_INT after enabling interrupts
    uint8_t statusInt = 0;
    if (readReg(QMI8658_REG_STATUS_INT, &statusInt) == ESP_OK) {
        ESP_LOGI(TAG, "STATUS_INT after enabling interrupts: 0x%02X", statusInt);
    } else {
        ESP_LOGW(TAG, "Failed to read STATUS_INT after enabling interrupts");
    }
    uint8_t ctrl1, ctrl2, ctrl3, ctrl7, ctrl8;
    readReg(QMI8658_REG_CTRL1, &ctrl1);
    readReg(QMI8658_REG_CTRL2, &ctrl2);
    readReg(QMI8658_REG_CTRL3, &ctrl3);
    readReg(QMI8658_REG_CTRL7, &ctrl7);
    readReg(QMI8658_REG_CTRL8, &ctrl8);

    ESP_LOGI(TAG, "CTRL1: 0x%02X, CTRL2: 0x%02X, CTRL3: 0x%02X, CTRL7: 0x%02X, CTRL8: 0x%02X", ctrl1, ctrl2, ctrl3, ctrl7, ctrl8);
    return ESP_OK;
}

// Cleanup sensor on termination
void Qmi8658C::onTerminate() const
{
    esp_err_t err = writeReg(QMI8658_REG_CTRL7, 0x00); // Disable all sensors
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to shut down sensor: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sensor shut down successfully");
    }
}

esp_err_t Qmi8658C::onSample(DataPoint &point)
{
    // We are called after a DRDY interrupt; perform a quick sanity check then read.
    uint8_t status = 0;
    uint8_t statusInt = 0;
    if (readReg(QMI8658_REG_STATUS0, &status) != ESP_OK || readReg(QMI8658_REG_STATUS_INT, &statusInt) != ESP_OK) {
        ESP_LOGE(TAG, "Status register read failed");
        return ESP_FAIL;
    }

    // If for some reason data isn't ready, bail out quickly.
    if (!((status & STATUS0_ACCEL_AVAIL) == STATUS0_ACCEL_AVAIL && (status & STATUS0_GYRO_AVAIL) == STATUS0_GYRO_AVAIL)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[12]; // 6 bytes for accel, 6 bytes for gyro
    if (readMultipleRegs(QMI8658_REG_AX_L, data, sizeof(data)) != ESP_OK) {
        ESP_LOGI(TAG, "Failed to read sensor data");
        return ESP_FAIL;
    }

    // Convert raw data to sensor values
    static int64_t lastTime = esp_timer_get_time();
    const int64_t currentTime = esp_timer_get_time();
    const float deltaTime = static_cast<float>(currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    populateRawAccelerometer(point, data);
    populateRawGyroscope(point, data);
    populateAcceleration(point, deltaTime);
    populateOrientation(point, deltaTime);
    populateTilt(point, deltaTime);
    return ESP_OK;
}

void Qmi8658C::populateAcceleration(DataPoint &point, const float &deltaTime)
{
    // Calculate angular acceleration as the rate of change of angular velocity
    point.acceleration.x = kalmanAccelerationX.estimate((point.gyroscope.x - lastGyro.x) / deltaTime, 0, deltaTime);
    point.acceleration.y = kalmanAccelerationY.estimate((point.gyroscope.y - lastGyro.y) / deltaTime, 0, deltaTime);
    point.acceleration.z = kalmanAccelerationZ.estimate((point.gyroscope.z - lastGyro.z) / deltaTime, 0, deltaTime);

    // Store the current gyroscope values as the last values for the next
    // iteration
    lastGyro = point.gyroscope;
}

void Qmi8658C::populateTilt(DataPoint &point, const float &deltaTime)
{
    constexpr float g = 9.81f; // Convert from m/s^2 to g units
    const float ax = point.accelerometer.x / g;
    const float ay = point.accelerometer.y / g;
    const float az = point.accelerometer.z / g;

    // Direct tilt calculation from accelerometer
    const float roll = std::atan2(ay, az);
    const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    // Use Kalman filters to smooth raw tilt values
    point.tilt.x = kalmanTiltRoll.estimate(roll * 180.0f / M_PI, 0, 0.0f); // Roll (degrees)
    point.tilt.y = kalmanTiltPitch.estimate(pitch * 180.0f / M_PI, 0,
                                            0.0f); // Pitch (degrees)

    point.tilt.z = 0; // Yaw cannot be determined from the accelerometer alone
}

void Qmi8658C::populateOrientation(DataPoint &point, const float &deltaTime)
{
    constexpr float ALPHA = 0.98f; // Complementary filter weight

    // Integrate gyro data to estimate angular changes
    lastOrientation.x += point.gyroscope.x * deltaTime; // Roll (X-axis)
    lastOrientation.y += point.gyroscope.y * deltaTime; // Pitch (Y-axis)
    lastOrientation.z += point.gyroscope.z * deltaTime; // Yaw (Z-axis)

    // Calculate pitch and roll from accelerometer (for tilt correction)
    const float accelPitch
        = std::atan2(point.accelerometer.x, std::sqrt(point.accelerometer.y * point.accelerometer.y + point.accelerometer.z * point.accelerometer.z));
    const float accelRoll
        = std::atan2(point.accelerometer.y, std::sqrt(point.accelerometer.x * point.accelerometer.x + point.accelerometer.z * point.accelerometer.z));

    // Use complementary filter to fuse gyro and accelerometer for roll and pitch
    lastOrientation.x = ALPHA * lastOrientation.x + (1 - ALPHA) * (accelRoll * 180.0f / M_PI);
    lastOrientation.y = ALPHA * lastOrientation.y + (1 - ALPHA) * (accelPitch * 180.0f / M_PI);

    // Apply Kalman filters to smooth roll, pitch, and yaw
    point.orientation.x = kalmanOrientationX.estimate(lastOrientation.x, point.gyroscope.x, deltaTime);
    point.orientation.y = kalmanOrientationY.estimate(lastOrientation.y, point.gyroscope.y, deltaTime);
    point.orientation.z = kalmanOrientationZ.estimate(lastOrientation.z, point.gyroscope.z, deltaTime);
}

void Qmi8658C::populateRawAccelerometer(DataPoint &point, const uint8_t *data)
{
    constexpr float ACCEL_SCALE{-2.0f * 9.81 / 32768.0f};                                                     // For ±2G range, in m/s^2
    point.accelerometer.x = static_cast<float>(static_cast<int16_t>((data[1] << 8) | data[0])) * ACCEL_SCALE; // X
    point.accelerometer.y = static_cast<float>(static_cast<int16_t>((data[3] << 8) | data[2])) * ACCEL_SCALE; // Y
    point.accelerometer.z = static_cast<float>(static_cast<int16_t>((data[5] << 8) | data[4])) * ACCEL_SCALE; // Z
}

void Qmi8658C::populateRawGyroscope(DataPoint &point, const uint8_t *data)
{
    constexpr float GYRO_SCALE{128.0f / 32768.0f};                                                         // For ±128DPS range
    point.gyroscope.x = static_cast<float>(static_cast<int16_t>((data[7] << 8) | data[6])) * GYRO_SCALE;   // X
    point.gyroscope.y = static_cast<float>(static_cast<int16_t>((data[9] << 8) | data[8])) * GYRO_SCALE;   // Y
    point.gyroscope.z = static_cast<float>(static_cast<int16_t>((data[11] << 8) | data[10])) * GYRO_SCALE; // Z
}

void Qmi8658C::terminate()
{
    if (eventGroup == nullptr) {
        return;
    }
    xEventGroupClearBits(eventGroup, IS_STOPPED);
    xEventGroupSetBits(eventGroup, DO_STOP);
    // Wait until task signals it stopped
    xEventGroupWaitBits(eventGroup, IS_STOPPED, pdTRUE, pdFALSE, portMAX_DELAY);

    // Cleanup GPIO ISR handlers
    gpio_isr_handler_remove(int1Pin);
    gpio_isr_handler_remove(int2Pin);

    // Delete event group
    vEventGroupDelete(eventGroup);
    eventGroup = nullptr;
    taskHandle = nullptr;
}

} // namespace Hilton