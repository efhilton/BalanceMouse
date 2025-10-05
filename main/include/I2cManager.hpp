#pragma once
#include <esp_err.h>
#include <driver/i2c_master.h>

namespace Hilton {
    class I2CManager {
        static constexpr auto TAG{"I2C_MANAGER"};
        const i2c_port_num_t i2CPortNumber;
        const gpio_num_t sdaPin{GPIO_NUM_MAX};
        const gpio_num_t sclPin{GPIO_NUM_MAX};

        // New driver handles
        i2c_master_bus_handle_t busHandle{nullptr};

    public:
        I2CManager(i2c_port_num_t i2CPortNumber, gpio_num_t sdaPin, gpio_num_t sclPin);

        ~I2CManager();

        esp_err_t initialize();

        esp_err_t terminate();

        // Getter for the bus handle to be used by other components
        [[nodiscard]] i2c_master_bus_handle_t getBusHandle() const { return busHandle; }
    };
}
