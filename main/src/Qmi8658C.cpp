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

// Initialize the sensor
esp_err_t Qmi8658C::initialize() {
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

  constexpr uint8_t ctrl8 = 0x40 | 0x02;
  err = writeReg(QMI8658_REG_CTRL8, ctrl8); // Map interrupt to INT1
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure CTRL8 (interrupt routing): %s",
             esp_err_to_name(err));
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

esp_err_t Qmi8658C::onSample(DataPoint &point) {
  uint8_t status;
  uint8_t statusInt;
  int64_t startTime = esp_timer_get_time();
  bool dataAvailable = false;

  while ((esp_timer_get_time() - startTime) <
         5'000'000) { // 5 seconds in microseconds
    if (readReg(QMI8658_REG_STATUS0, &status) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read status register");
      return ESP_FAIL;
    }
    if (readReg(QMI8658_REG_STATUS_INT, &statusInt) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read status_int register");
      return ESP_FAIL;
    }

    if ((status & STATUS0_ACCEL_AVAIL) == STATUS0_ACCEL_AVAIL &&
        (status & STATUS0_GYRO_AVAIL) == STATUS0_GYRO_AVAIL &&
        (statusInt & 0x01) == 0x01) {
      dataAvailable = true;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent busy waiting
  }

  if (!dataAvailable) {
    return ESP_ERR_TIMEOUT; // No new data available within timeout
  }

  uint8_t data[12]; // 6 bytes for accel, 6 bytes for gyro
  if (readMultipleRegs(QMI8658_REG_AX_L, data, sizeof(data)) != ESP_OK) {
    ESP_LOGI(TAG, "Failed to read sensor data");
    return ESP_FAIL;
  }

  // Convert raw data to sensor values
  static int64_t lastTime = esp_timer_get_time();
  const int64_t currentTime = esp_timer_get_time();
  const float deltaTime =
      static_cast<float>(currentTime - lastTime) / 1000000.0f;
  lastTime = currentTime;

  populateRawAccelerometer(point, data);
  populateRawGyroscope(point, data);
  populateAcceleration(point, deltaTime);
  populateOrientation(point, deltaTime);
  populateTilt(point, deltaTime);
  return ESP_OK;
}

void Qmi8658C::populateAcceleration(DataPoint &point, const float &deltaTime) {
  // Calculate angular acceleration as the rate of change of angular velocity
  point.acceleration.x = kalmanAccelerationX.estimate(
      (point.gyroscope.x - lastGyro.x) / deltaTime, 0, deltaTime);
  point.acceleration.y = kalmanAccelerationY.estimate(
      (point.gyroscope.y - lastGyro.y) / deltaTime, 0, deltaTime);
  point.acceleration.z = kalmanAccelerationZ.estimate(
      (point.gyroscope.z - lastGyro.z) / deltaTime, 0, deltaTime);

  // Store the current gyroscope values as the last values for the next
  // iteration
  lastGyro = point.gyroscope;
}

void Qmi8658C::populateTilt(DataPoint &point, const float &deltaTime) {
  constexpr float g = 9.81f; // Convert from m/s^2 to g units
  const float ax = point.accelerometer.x / g;
  const float ay = point.accelerometer.y / g;
  const float az = point.accelerometer.z / g;

  // Direct tilt calculation from accelerometer
  const float roll = std::atan2(ay, az);
  const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

  // Use Kalman filters to smooth raw tilt values
  point.tilt.x =
      kalmanTiltRoll.estimate(roll * 180.0f / M_PI, 0, 0.0f); // Roll (degrees)
  point.tilt.y = kalmanTiltPitch.estimate(pitch * 180.0f / M_PI, 0,
                                          0.0f); // Pitch (degrees)

  point.tilt.z = 0; // Yaw cannot be determined from the accelerometer alone
}

void Qmi8658C::populateOrientation(DataPoint &point, const float &deltaTime) {
  constexpr float ALPHA = 0.98f; // Complementary filter weight

  // Integrate gyro data to estimate angular changes
  lastOrientation.x += point.gyroscope.x * deltaTime; // Roll (X-axis)
  lastOrientation.y += point.gyroscope.y * deltaTime; // Pitch (Y-axis)
  lastOrientation.z += point.gyroscope.z * deltaTime; // Yaw (Z-axis)

  // Calculate pitch and roll from accelerometer (for tilt correction)
  const float accelPitch =
      std::atan2(point.accelerometer.x,
                 std::sqrt(point.accelerometer.y * point.accelerometer.y +
                           point.accelerometer.z * point.accelerometer.z));
  const float accelRoll =
      std::atan2(point.accelerometer.y,
                 std::sqrt(point.accelerometer.x * point.accelerometer.x +
                           point.accelerometer.z * point.accelerometer.z));

  // Use complementary filter to fuse gyro and accelerometer for roll and pitch
  lastOrientation.x =
      ALPHA * lastOrientation.x + (1 - ALPHA) * (accelRoll * 180.0f / M_PI);
  lastOrientation.y =
      ALPHA * lastOrientation.y + (1 - ALPHA) * (accelPitch * 180.0f / M_PI);

  // Apply Kalman filters to smooth roll, pitch, and yaw
  point.orientation.x = kalmanOrientationX.estimate(
      lastOrientation.x, point.gyroscope.x, deltaTime);
  point.orientation.y = kalmanOrientationY.estimate(
      lastOrientation.y, point.gyroscope.y, deltaTime);
  point.orientation.z = kalmanOrientationZ.estimate(
      lastOrientation.z, point.gyroscope.z, deltaTime);
}

void Qmi8658C::populateRawAccelerometer(DataPoint &point, const uint8_t *data) {
  constexpr float ACCEL_SCALE{-2.0f * 9.81 /
                              32768.0f}; // For ±2G range, in m/s^2
  point.accelerometer.x =
      static_cast<float>(static_cast<int16_t>((data[1] << 8) | data[0])) *
      ACCEL_SCALE; // X
  point.accelerometer.y =
      static_cast<float>(static_cast<int16_t>((data[3] << 8) | data[2])) *
      ACCEL_SCALE; // Y
  point.accelerometer.z =
      static_cast<float>(static_cast<int16_t>((data[5] << 8) | data[4])) *
      ACCEL_SCALE; // Z
}

void Qmi8658C::populateRawGyroscope(DataPoint &point, const uint8_t *data) {
  constexpr float GYRO_SCALE{128.0f / 32768.0f}; // For ±128DPS range
  point.gyroscope.x =
      static_cast<float>(static_cast<int16_t>((data[7] << 8) | data[6])) *
      GYRO_SCALE; // X
  point.gyroscope.y =
      static_cast<float>(static_cast<int16_t>((data[9] << 8) | data[8])) *
      GYRO_SCALE; // Y
  point.gyroscope.z =
      static_cast<float>(static_cast<int16_t>((data[11] << 8) | data[10])) *
      GYRO_SCALE; // Z
}
} // namespace Hilton