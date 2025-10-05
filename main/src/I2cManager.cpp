#include <esp_log.h>
#include <I2cManager.hpp>
#include <driver/i2c_master.h>

namespace Hilton {
    I2CManager::I2CManager(const int i2CPortNumber, const gpio_num_t sdaPin, const gpio_num_t sclPin)
        : i2CPortNumber(i2CPortNumber)
          , sdaPin(sdaPin)
          , sclPin(sclPin) {
    }

    I2CManager::~I2CManager() = default;

    esp_err_t I2CManager::initialize() {
        if (busHandle != nullptr) {
            return ESP_OK;
        }

        // Configure I2C primary bus
        i2c_master_bus_config_t busConfig = {};
        busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
        busConfig.i2c_port = i2CPortNumber;
        busConfig.scl_io_num = sclPin;
        busConfig.sda_io_num = sdaPin;
        busConfig.glitch_ignore_cnt = 7;
        busConfig.flags.enable_internal_pullup = true;

        const esp_err_t err = i2c_new_master_bus(&busConfig, &busHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C master bus creation failed: %s", esp_err_to_name(err));
            return err;
        }

        ESP_LOGI(TAG, "I2C master bus initialized on port %d", i2CPortNumber);
        return ESP_OK;
    }

    esp_err_t I2CManager::terminate() {
        if (busHandle == nullptr) {
            return ESP_OK;
        }

        esp_err_t err = ESP_OK;
        err = i2c_del_master_bus(busHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C master bus deletion failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "I2C master bus terminated successfully");
        }
        busHandle = nullptr;

        return err;
    }
}
