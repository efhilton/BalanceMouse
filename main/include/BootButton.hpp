#pragma once

#include <ButtonPressDetector.hpp>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

namespace Hilton {
    /**
     * BOOT button (GPIO0 on the ESP32-S3-Matrix: 10k external pull-up, pressed = LOW).
     *
     * GPIO0 is configured as an input with the internal pull-up enabled and an
     * any-edge interrupt. The ISR only wakes a small task; the task feeds the raw
     * level into ButtonPressDetector (30 ms debounce) and, while the button is held,
     * polls every 10 ms so the long-press action fires AT the 3 s mark without
     * waiting for release. Handlers run on that task, never in the ISR.
     */
    class BootButton {
    public:
        using Handler = std::function<void()>;

        static constexpr auto TAG{"BOOT_BUTTON"};
        static constexpr uint32_t POLL_INTERVAL_MS{10};

        BootButton(gpio_num_t pin, Handler onShortPress, Handler onLongPress);

        esp_err_t start();

    private:
        static void isrHandler(void *arg);

        static void taskEntry(void *arg);

        void run();

        gpio_num_t pin;
        Handler onShortPress;
        Handler onLongPress;
        ButtonPressDetector detector;
        TaskHandle_t task{nullptr};
    };
} // namespace Hilton
