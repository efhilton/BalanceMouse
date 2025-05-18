
#include "I2cManager.hpp"

#include "driver/i2c.h"
#include "esp_log.h"

namespace Hilton {

I2cManager::I2cManager(i2c_port_t i2cPort, gpio_num_t sdaPin, gpio_num_t sclPin)
    : i2cPort(i2cPort)
    , sdaPin(sdaPin)
    , sclPin(sclPin)
{}

I2cManager::~I2cManager() = default;

esp_err_t I2cManager::initialize()
{
    if (initialized) {
        return ESP_OK;
    }
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdaPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = sclPin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    esp_err_t err = i2c_param_config(i2cPort, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return err;
    }

    err = i2c_driver_install(i2cPort, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed");
        return err;
    }

    initialized = true;
    return err;
}
esp_err_t I2cManager::terminate()
{
    if (!initialized) {
        return ESP_OK;
    }
    esp_err_t err = i2c_driver_delete(i2cPort);
    initialized = false;
    return err;
}
} // namespace Hilton