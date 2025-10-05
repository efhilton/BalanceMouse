#include <LedController.hpp>
#include <driver/gpio.h>

namespace Hilton {
    LedController::LedController(const gpio_num_t ledPin) : controlPin(ledPin) {
    }

    LedController::~LedController() = default;
}
