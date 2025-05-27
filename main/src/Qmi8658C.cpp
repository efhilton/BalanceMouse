#include <Qmi8658C.hpp>
#include <cstring>
#include <ctgmath>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace Hilton {

esp_err_t Qmi8658C::writeReg(const uint8_t reg, const uint8_t data) const {
  uint8_t write_buf[2] = {reg, data};
  return i2c_master_write_to_device(i2cPort, QMI8658_L_SLAVE_ADDRESS, write_buf,
                                    sizeof(write_buf), pdMS_TO_TICKS(100));
}

esp_err_t Qmi8658C::readReg(const uint8_t reg, uint8_t *data) const {
  return i2c_master_write_read_device(i2cPort, QMI8658_L_SLAVE_ADDRESS, &reg, 1,
                                      data, 1, pdMS_TO_TICKS(100));
}

esp_err_t Qmi8658C::readMultipleRegs(const uint8_t startReg, uint8_t *data,
                                     const size_t len) const {
  return i2c_master_write_read_device(i2cPort, QMI8658_L_SLAVE_ADDRESS,
                                      &startReg, 1, data, len,
                                      pdMS_TO_TICKS(100));
}

Qmi8658C::Qmi8658C(const i2c_port_t i2CPort) : i2cPort(i2CPort) {}

esp_err_t Qmi8658C::initialize() const {
  uint8_t whoami;
  if (readReg(QMI8658_REG_WHOAMI, &whoami) != ESP_OK ||
      whoami != QMI8658_REG_WHOAMI_DEFAULT) {
    ESP_LOGI(TAG, "Failed to identify QMI8658C sensor");
    return ESP_ERR_INVALID_STATE;
  }

  // Add soft reset
  esp_err_t err = writeReg(QMI8658_REG_RESET, QMI8658_REG_RESET_DEFAULT);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset sensor");
    return err;
  }

  // Wait for reset to complete
  vTaskDelay(pdMS_TO_TICKS(10));

  err = configure();
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "Failed to configure QMI8658C sensor: %s",
             esp_err_to_name(err));
    return err;
  }
  ESP_LOGI(TAG, "Initialized QMI8658C sensor");
  return err;
}

esp_err_t Qmi8658C::disableAllSensors() const {
  // Reset and initialize the sensor
  const esp_err_t err =
      writeReg(QMI8658_REG_CTRL7, 0x00); // Disable all sensors first
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset sensor: %s", esp_err_to_name(err));
    return err;
  }
  return err;
}

esp_err_t Qmi8658C::configure() const {
  esp_err_t err = disableAllSensors();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable accelerometer: %s", esp_err_to_name(err));
    return err;
  }

  // First configure CTRL1 - Enable timestamp (bit 6) and enable motion (bit 5)
  constexpr uint8_t ctrl1Value{BIT5 | BIT6};
  err = writeReg(QMI8658_REG_CTRL1, ctrl1Value);
  if (err != ESP_OK) {
    return err;
  }

  // Configure accelerometer (31.25Hz ODR, +/- 2g full scale, no self test)
  // Bits[7:5]: Full Scale Range (FSR)
  // Bits[4:2]: Output Data Rate (ODR)
  // Bits[1:0]: Low-pass filter bandwidth
  constexpr uint8_t accelConfigValue{0b00001000};
  // 000 -> ±2g range (correct)
  // 001 -> 31.25Hz ODR (correct)
  // 00  -> Narrowest bandwidth (fine)

  err = writeReg(QMI8658_REG_CTRL2, accelConfigValue); // 0b0 000 1000
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure accelerometer: %s",
             esp_err_to_name(err));
    return err;
  }

  // Configure gyroscope (±128°/s range, 29.375 Hz ODR)
  // Bits[7:5]: Full Scale Range (FSR)
  // Bits[4:2]: Output Data Rate (ODR)
  // Bits[1:0]: Low-pass filter bandwidth
  constexpr uint8_t gyroConfigValue{0b00111000};
  // 001 -> ±128 dps range (correct)
  // 110 -> Higher ODR than accelerometer (should match accel)
  // 00  -> Narrowest bandwidth (fine)

  err = writeReg(QMI8658_REG_CTRL3, gyroConfigValue); // 0b01100011
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(err));
    return err;
  }

  // // Enable data ready interrupt
  // constexpr uint8_t motionDetectionValue{0x00};
  // err = writeReg(QMI8658_REG_CTRL8, motionDetectionValue);
  // if (err != ESP_OK) {
  //     ESP_LOGE(TAG, "Failed to enable data ready interrupt: %s",
  //     esp_err_to_name(err)); return err;
  // }

  // Enable both accelerometer and gyroscope
  err =
      writeReg(QMI8658_REG_CTRL7, QMI8658_ACCEL_EN_MASK | QMI8658_GYRO_EN_MASK);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable sensors: %s", esp_err_to_name(err));
    return err;
  }

  uint8_t ctrl1, ctrl2, ctrl3, ctrl7, ctrl8;
  readReg(QMI8658_REG_CTRL1, &ctrl1);
  readReg(QMI8658_REG_CTRL2, &ctrl2);
  readReg(QMI8658_REG_CTRL3, &ctrl3);
  readReg(QMI8658_REG_CTRL7, &ctrl7);
  readReg(QMI8658_REG_CTRL8, &ctrl8);

  ESP_LOGI(TAG,
           "After config\n\tCTRL1: 0x%02X\n\tCTRL2: 0x%02X\n\tCTRL3: "
           "0x%02X\n\tCTRL7: "
           "0x%02X\n\tCTRL8: 0x%02X",
           ctrl1, ctrl2, ctrl3, ctrl7, ctrl8);

  vTaskDelay(pdMS_TO_TICKS(10));

  return ESP_OK;
}

void Qmi8658C::onSample() {
  uint8_t status;
  if (readReg(QMI8658_REG_STATUS0, &status) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read status register");
    return;
  }

  if (!(status & (STATUS0_ACCEL_AVAIL | STATUS0_GYRO_AVAIL))) {
    return; // No new data available
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
  DataPoint point{.accelerometer =
                      {
                          static_cast<float>(
                              static_cast<int16_t>((data[1] << 8) | data[0])) *
                              ACCEL_SCALE, // X
                          static_cast<float>(
                              static_cast<int16_t>((data[3] << 8) | data[2])) *
                              ACCEL_SCALE, // Y
                          static_cast<float>(
                              static_cast<int16_t>((data[5] << 8) | data[4])) *
                              ACCEL_SCALE // Z
                      },
                  .gyroscope =
                      {
                          static_cast<float>(
                              static_cast<int16_t>((data[7] << 8) | data[6])) *
                              GYRO_SCALE, // X
                          static_cast<float>(
                              static_cast<int16_t>((data[9] << 8) | data[8])) *
                              GYRO_SCALE, // Y
                          static_cast<float>(static_cast<int16_t>(
                              (data[11] << 8) | data[10])) *
                              GYRO_SCALE // Z
                      },
                  .acceleration{0, 0, 0},
                  .orientation{0, 0, 0},
                  .tilt{0, 0, 0}};

  populateAcceleration(point);
  populateOrientation(point);
  populateTilt(point);

  dataCallbackService.notify(point);
}

void Qmi8658C::onTerminate() const {
  // Put the sensor in sleep mode
  esp_err_t err = writeReg(QMI8658_REG_CTRL7, 0x00);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to put sensor in sleep mode: %s. Ignored.",
             esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "Terminated QMI8658C sensor");
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
  const float accelPitch = atan2(
      point.accelerometer.x,
      static_cast<float>(sqrt(point.accelerometer.y * point.accelerometer.y +
                              point.accelerometer.z * point.accelerometer.z)));
  const float accelRoll = atan2(
      point.accelerometer.y,
      static_cast<float>(sqrt(point.accelerometer.x * point.accelerometer.x +
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