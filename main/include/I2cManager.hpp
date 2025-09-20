#pragma once
#include "esp_err.h"
#include "hal/i2c_types.h"

#include <driver/gpio.h>

namespace Hilton {
class I2cManager
{
    static constexpr auto TAG{"I2C_MANAGER"};
    i2c_port_t i2cPort{I2C_NUM_MAX};
    gpio_num_t sdaPin{GPIO_NUM_MAX};
    gpio_num_t sclPin{GPIO_NUM_MAX};
    bool initialized{false};

public:
    I2cManager(i2c_port_t i2cPort, gpio_num_t sdaPin, gpio_num_t sclPin);
    ~I2cManager();

    esp_err_t initialize();
    esp_err_t terminate();
};
} // namespace Hilton