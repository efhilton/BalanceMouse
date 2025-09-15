#include "Qmi8658C.hpp"
#include "esp_log.h"

#include <cmath>
#include <cstring>
#include <driver/i2c.h>
#include <esp_timer.h>

namespace Hilton {

constexpr const char *TAG = "QMI8658C";

// Constructor
Qmi8658C::Qmi8658C(const i2c_port_t i2CPort) : i2cPort(i2CPort) {}

// Simplified helper to write a single register
esp_err_t Qmi8658C::writeReg(uint8_t reg, uint8_t data) const {
  uint8_t buffer[2] = {reg, data};
  esp_err_t err =
      i2c_master_write_to_device(i2cPort, QMI8658_L_SLAVE_ADDRESS, buffer,
                                 sizeof(buffer), pdMS_TO_TICKS(1000));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg,
             esp_err_to_name(err));
  }
  return err;
}

// Simplified helper to read a single register
esp_err_t Qmi8658C::readReg(uint8_t reg, uint8_t *data) const {
  esp_err_t err = i2c_master_write_read_device(
      i2cPort, QMI8658_L_SLAVE_ADDRESS, &reg, 1, data, 1, pdMS_TO_TICKS(1000));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg,
             esp_err_to_name(err));
  }
  return err;
}

// Simplified helper to read multiple registers starting at a given address
esp_err_t Qmi8658C::readMultipleRegs(uint8_t startReg, uint8_t *buffer,
                                     size_t len) const {
  esp_err_t err =
      i2c_master_write_read_device(i2cPort, QMI8658_L_SLAVE_ADDRESS, &startReg,
                                   1, buffer, len, pdMS_TO_TICKS(1000));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read multiple registers starting at 0x%02X: %s",
             startReg, esp_err_to_name(err));
  }
  return err;
}

// Enable data-ready interrupt and map it to INT1
esp_err_t Qmi8658C::enable_interrupt() const {
  esp_err_t err = writeReg(QMI8658_REG_CTRL8, 0x42); // Map interrupt to INT1
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure CTRL8 (interrupt routing): %s",
             esp_err_to_name(err));
  }
  return err;
}

// Initialize the sensor
esp_err_t Qmi8658C::initialize() const {
  uint8_t ctrl1, ctrl2, ctrl3, ctrl7, ctrl8;
  readReg(QMI8658_REG_CTRL1, &ctrl1);
  readReg(QMI8658_REG_CTRL2, &ctrl2);
  readReg(QMI8658_REG_CTRL3, &ctrl3);
  readReg(QMI8658_REG_CTRL7, &ctrl7);
  readReg(QMI8658_REG_CTRL8, &ctrl8);

  ESP_LOGI(TAG,
           "BEFORE Configured Sensor:\nCTRL1: 0x%02X\nCTRL2: 0x%02X\nCTRL3: "
           "0x%02X\nCTRL7: 0x%02X\nCTRL8: 0x%02X",
           ctrl1, ctrl2, ctrl3, ctrl7, ctrl8);

  uint8_t whoami;
  if (readReg(QMI8658_REG_WHOAMI, &whoami) != ESP_OK ||
      whoami != QMI8658_REG_WHOAMI_DEFAULT) {
    ESP_LOGE(TAG,
             "Device not found or incorrect WHOAMI register value (0x%02X)",
             whoami);
    return ESP_ERR_INVALID_STATE;
  }

  // Reset sensor
  esp_err_t err = writeReg(QMI8658_REG_RESET, QMI8658_REG_RESET_DEFAULT);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Sensor reset failed: %s", esp_err_to_name(err));
    return err;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // Allow reset to complete

  // Perform initial configuration
  err = configure();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Sensor configuration failed: %s", esp_err_to_name(err));
    return err;
  }

  // Enable interrupts
  err = enable_interrupt();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Interrupt configuration failed: %s", esp_err_to_name(err));
  }

  readReg(QMI8658_REG_CTRL1, &ctrl1);
  readReg(QMI8658_REG_CTRL2, &ctrl2);
  readReg(QMI8658_REG_CTRL3, &ctrl3);
  readReg(QMI8658_REG_CTRL7, &ctrl7);
  readReg(QMI8658_REG_CTRL8, &ctrl8);

  ESP_LOGI(TAG,
           "AFTER Configured Sensor:\nCTRL1: 0x%02X\nCTRL2: 0x%02X\nCTRL3: "
           "0x%02X\nCTRL7: 0x%02X\nCTRL8: 0x%02X",
           ctrl1, ctrl2, ctrl3, ctrl7, ctrl8);

  return err;
}

// Disable all sensors
esp_err_t Qmi8658C::disableAllSensors() const {
  return writeReg(QMI8658_REG_CTRL7,
                  0x00); // Disable both accelerometer and gyroscope
}

// Configure the sensor settings
esp_err_t Qmi8658C::configure() const {
  esp_err_t err = disableAllSensors();
  if (err != ESP_OK)
    return err;

  // Configure accelerometer
  constexpr uint8_t accelConfig = 0b00000111; // ±2g, 58.75kHz
  err = writeReg(QMI8658_REG_CTRL2, accelConfig);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure accelerometer: %s",
             esp_err_to_name(err));
    return err;
  }

  // Configure gyroscope
  constexpr uint8_t gyroConfig = 0b00110111; // ±128dps, 29.375Hz
  err = writeReg(QMI8658_REG_CTRL3, gyroConfig);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(err));
    return err;
  }

  // Enable both accelerometer and gyroscope
  constexpr uint8_t ctrl7 = QMI8658_ACCEL_EN_MASK | QMI8658_GYRO_EN_MASK;
  err = writeReg(QMI8658_REG_CTRL7, ctrl7);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable sensors: %s", esp_err_to_name(err));
  }

  return err;
}

// Cleanup sensor on termination
void Qmi8658C::onTerminate() const {
  esp_err_t err = writeReg(QMI8658_REG_CTRL7, 0x00); // Disable all sensors
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to shut down sensor: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Sensor shut down successfully");
  }
}

void Qmi8658C::onSample(DataPoint &point) {
  uint8_t status;
  int64_t startTime = esp_timer_get_time();
  bool dataAvailable = false;

  while ((esp_timer_get_time() - startTime) <
         5000000) { // 5 seconds in microseconds
    if (readReg(QMI8658_REG_STATUS0, &status) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read status register");
      return;
    }

    if ((status & STATUS0_ACCEL_AVAIL) == STATUS0_ACCEL_AVAIL ||
        (status & STATUS0_GYRO_AVAIL) == STATUS0_GYRO_AVAIL) {
      dataAvailable = true;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent busy waiting
  }

  if (!dataAvailable) {
    return; // No new data available within timeout
  }

  uint8_t data[12]; // 6 bytes for accel, 6 bytes for gyro
  if (readMultipleRegs(QMI8658_REG_AX_L, data, sizeof(data)) != ESP_OK) {
    ESP_LOGI(TAG, "Failed to read sensor data");
    return;
  }

  // Convert raw data to sensor values
  constexpr float ACCEL_SCALE{-2.0f * 9.81 /
                              32768.0f};         // For ±2G range, in m/s^2
  constexpr float GYRO_SCALE{128.0f / 32768.0f}; // For ±128DPS range
  point.accelerometer.x =
      static_cast<float>(static_cast<int16_t>((data[1] << 8) | data[0])) *
      ACCEL_SCALE; // X
  point.accelerometer.y =
      static_cast<float>(static_cast<int16_t>((data[3] << 8) | data[2])) *
      ACCEL_SCALE; // Y
  point.accelerometer.z =
      static_cast<float>(static_cast<int16_t>((data[5] << 8) | data[4])) *
      ACCEL_SCALE; // Z

  point.gyroscope.x =
      static_cast<float>(static_cast<int16_t>((data[7] << 8) | data[6])) *
      GYRO_SCALE; // X
  point.gyroscope.y =
      static_cast<float>(static_cast<int16_t>((data[9] << 8) | data[8])) *
      GYRO_SCALE; // Y
  point.gyroscope.z =
      static_cast<float>(static_cast<int16_t>((data[11] << 8) | data[10])) *
      GYRO_SCALE; // Z

  point.acceleration = {0, 0, 0};
  point.orientation = {0, 0, 0};
  point.tilt = {0, 0, 0};

  populateAcceleration(point);
  populateOrientation(point);
  populateTilt(point);
}

void Qmi8658C::populateAcceleration(DataPoint &point) {
  static int64_t lastTime = esp_timer_get_time();
  const int64_t currentTime = esp_timer_get_time();
  const float deltaTime =
      static_cast<float>(currentTime - lastTime) / 1000000.0f;
  lastTime = currentTime;

  // Calculate angular acceleration as the rate of change of angular velocity
  point.acceleration.x = (point.gyroscope.x - lastGyro.x) / deltaTime;
  point.acceleration.y = (point.gyroscope.y - lastGyro.y) / deltaTime;
  point.acceleration.z = (point.gyroscope.z - lastGyro.z) / deltaTime;

  // Update last gyroscope values for next iteration
  lastGyro = point.gyroscope;
}

void Qmi8658C::populateTilt(DataPoint &point) {
  constexpr float g = 9.81f; // Convert from m/s^2 to g units
  const float ax = point.accelerometer.x / g;
  const float ay = point.accelerometer.y / g;
  const float az = point.accelerometer.z / g;

  // Calculate roll angle (rotation around X-axis)
  point.tilt.x = atan2(ay, az);

  // Calculate pitch angle (rotation around Y-axis)
  point.tilt.y = atan2(-ax, sqrt(ay * ay + az * az));

  // Convert radians to degrees
  point.tilt.x *= 180.0f / M_PI;
  point.tilt.y *= 180.0f / M_PI;
  point.tilt.z = 0; // Can't determine yaw from accelerometer alone
}

void Qmi8658C::populateOrientation(DataPoint &point) {
  constexpr float ALPHA = 0.98f;
  static int64_t lastTime = esp_timer_get_time();
  const int64_t currentTime = esp_timer_get_time();
  const float deltaTime =
      static_cast<float>(currentTime - lastTime) / 1000000.0f;
  lastTime = currentTime;

  // Integrate gyro data to get angles
  orientation.x += point.gyroscope.x * deltaTime;
  orientation.y += point.gyroscope.y * deltaTime;
  orientation.z += point.gyroscope.z * deltaTime;

  // Use accelerometer to estimate tilt (roll and pitch)
  const float accelPitch =
      std::atan2(point.accelerometer.x,
                 static_cast<float>(
                     std::sqrt(point.accelerometer.y * point.accelerometer.y +
                               point.accelerometer.z * point.accelerometer.z)));
  const float accelRoll =
      std::atan2(point.accelerometer.y,
                 static_cast<float>(
                     std::sqrt(point.accelerometer.x * point.accelerometer.x +
                               point.accelerometer.z * point.accelerometer.z)));

  // Correct gyroscope orientation using the accelerometer (complementary
  // filter)
  orientation.x = ALPHA * orientation.x + (1 - ALPHA) * accelPitch;
  orientation.y = ALPHA * orientation.y + (1 - ALPHA) * accelRoll;

  point.orientation.x = orientation.x;
  point.orientation.y = orientation.y;
  point.orientation.z = orientation.z;
}
} // namespace Hilton