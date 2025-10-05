#pragma once
#include <driver/gpio.h>

namespace Hilton {
    class LedController {
    public:
        explicit LedController(gpio_num_t ledPin);

        ~LedController();

    private:
        static constexpr auto TAG{"LedController"};
        const gpio_num_t controlPin;
    };
} // Hilton