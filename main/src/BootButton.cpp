#include <BootButton.hpp>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace Hilton {
    BootButton::BootButton(const gpio_num_t pin, Handler onShortPress, Handler onLongPress)
        : pin(pin), onShortPress(std::move(onShortPress)), onLongPress(std::move(onLongPress)) {
    }

    esp_err_t BootButton::start() {
        const BaseType_t created = xTaskCreate(&BootButton::taskEntry, "boot_button", 3072, this, 4, &task);
        if (created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task");
            return ESP_FAIL;
        }

        gpio_config_t cfg{};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_ANYEDGE;
        esp_err_t err = gpio_config(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
            return err;
        }

        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // INVALID_STATE: already installed
            ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
            return err;
        }
        err = gpio_isr_handler_add(pin, &BootButton::isrHandler, this);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
            return err;
        }
        // Kick once so a button already held at boot is seen.
        xTaskNotifyGive(task);
        ESP_LOGI(TAG, "BOOT button on GPIO%d: short press = re-zero, hold %lums = forget bond + pair",
                 pin, static_cast<unsigned long>(ButtonPressDetector::DEFAULT_LONG_PRESS_MS));
        return ESP_OK;
    }

    void IRAM_ATTR BootButton::isrHandler(void *arg) {
        auto *self = static_cast<BootButton *>(arg);
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        if (self->task != nullptr) {
            vTaskNotifyGiveFromISR(self->task, &higherPriorityTaskWoken);
        }
        if (higherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }

    void BootButton::taskEntry(void *arg) {
        static_cast<BootButton *>(arg)->run();
    }

    void BootButton::run() {
        while (true) {
            // Sleep until an edge interrupt; while held or bouncing, poll so the
            // debounce window and the 3 s long-press deadline are honoured.
            const TickType_t wait = detector.needsPolling() ? pdMS_TO_TICKS(POLL_INTERVAL_MS) : portMAX_DELAY;
            ulTaskNotifyTake(pdTRUE, wait == 0 ? 1 : wait);

            const bool pressed = gpio_get_level(pin) == 0;
            const auto now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            switch (detector.update(pressed, now)) {
                case ButtonPressDetector::Event::ShortPress:
                    ESP_LOGI(TAG, "Short press");
                    if (onShortPress) onShortPress();
                    break;
                case ButtonPressDetector::Event::LongPress:
                    ESP_LOGW(TAG, "Long press");
                    if (onLongPress) onLongPress();
                    break;
                case ButtonPressDetector::Event::None:
                    break;
            }
        }
    }
} // namespace Hilton
