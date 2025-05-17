#include <cstdio>

#include "FreeRTOSConfig.h"
#include "Qmi8658C.hpp"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
static constexpr auto TAG{"BALANCE_BOARD"};

esp_err_t InitializeI2C(i2c_port_t i2cNum = I2C_NUM_0)
{
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE == 1;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE == 1;
    conf.master.clk_speed = 100000;

    esp_err_t err = i2c_param_config(i2cNum, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return err;
    }

    err = i2c_driver_install(i2cNum, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed");
        return err;
    }
    return i2cNum;
}

void shutdownI2C(const i2c_port_t i2_c_port)
{
    i2c_driver_delete(i2_c_port);
}

extern "C" void app_main(void)
{
    constexpr i2c_port_t i2cNum = I2C_NUM_0;
    const esp_err_t err = InitializeI2C(i2cNum);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
        return;
    }

    Hilton::SimpleNotificationService<Hilton::Qmi8658C::DataPoint>::OnDataCallback onData{
        [](const Hilton::Qmi8658C::DataPoint &data) {
            ESP_LOGI(
                TAG,
                "%02d, %02d, %02d, %02d, %02d, %02d, %02d, %02d, %d02",
                data.accelerometer.x,
                data.accelerometer.y,
                data.accelerometer.z,
                data.gyroscope.x,
                data.gyroscope.y,
                data.gyroscope.z,
                data.orientation.x,
                data.orientation.y,
                data.orientation.z);
        }};

    Hilton::SimpleNotificationService<std::string>::OnDataCallback onLog{
        [](const std::string &log) { ESP_LOGI(TAG, "LOG: %s", log.c_str()); }};

    ESP_LOGI(TAG, "I2C initialized in %d", i2cNum);
    Hilton::Qmi8658C qmi8658c(i2cNum);
    qmi8658c.dataCallbackService.addCallback(onData);
    qmi8658c.logCallbackService.addCallback(onLog);

    esp_err_t init_result = qmi8658c.initialize();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize QMI8658C: %s", esp_err_to_name(init_result));
        shutdownI2C(i2cNum);
        return;
    }

    // Create a task for handling sensor data
    xTaskCreate(
        [](void *arg) {
            auto *sensor = static_cast<Hilton::Qmi8658C *>(arg);
            while (true) {
                sensor->onSample();
                // Wait for next sample. The delay should match the configured ODR
                // For 1000Hz ODR, we wait 1ms
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        },
        "sensor_task",
        4096,
        &qmi8658c,
        5,
        nullptr);

    // Main task can do other things or just wait
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task heartbeat
    }
}