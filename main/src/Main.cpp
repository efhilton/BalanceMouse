#include <cstdio>

#include "QMI8658C.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/i2c_types.h"
#include "hal/gpio_types.h"
static constexpr auto TAG{"BALANCE_BOARD"};

esp_err_t InitializeI2C(i2c_port_t i2cNum = I2C_NUM_0) {
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

void shutdownI2c(const i2c_port_t i2_c_port) {
    i2c_driver_delete(i2_c_port);
}

void onDataCallback(const uint8_t *data, const size_t len) {

}

extern "C" void app_main(void) {
    constexpr i2c_port_t i2cNum = I2C_NUM_0;
    const esp_err_t err = InitializeI2C(i2cNum);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C initialized in %d", i2cNum);
    QMI8658C qmi8658c(i2cNum,onDataCallback);
    qmi8658c.initialize();
    while (true) {
        qmi8658c.sample();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    qmi8658c.onTerminate();
    shutdownI2c(i2cNum);
}
