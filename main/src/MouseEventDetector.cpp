#include <MouseEventDetector.hpp>
#include <memory>
#include <string>
#include <Qmi8658C.hpp>

#include "Finalizer.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "soc/gpio_num.h"

namespace Hilton {

enum DetectorEventMask { SHUTDOWN_EVENT = BIT0, DATA_AVAILABLE = BIT1 };
void MouseEventDetector::gpioIsrHandler(void *p)
{
    const auto *self = static_cast<MouseEventDetector *>(p);
    xEventGroupSetBitsFromISR(self->eventGroup, DATA_AVAILABLE, nullptr);
}

MouseEventDetector::MouseEventDetector(
    const std::shared_ptr<Qmi8658C> &qmi8658C, const gpio_num_t int1Pin, const gpio_num_t int2Pin)
    : qmi8658c(qmi8658C)
    , onDataCallback([this](const Qmi8658C::DataPoint &data) { analyze(data); })
    , int1Pin(int1Pin)
    , int2Pin(int2Pin)
    , eventGroup(xEventGroupCreate())
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << int1Pin) | (1ULL << int2Pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(int1Pin, gpioIsrHandler, this);
    gpio_isr_handler_add(int2Pin, gpioIsrHandler, this);
}

void MouseEventDetector::run()
{
    static bool isRunning{false};

    if (isRunning) {
        return;
    }
    // Create a task for handling sensor data
    xTaskCreate(
        [](void *arg) {
            auto *self = static_cast<MouseEventDetector *>(arg);
            int id = self->qmi8658c->dataCallbackService.addCallback(self->onDataCallback);
            Finalizer f{[&self, &id] {
                self->qmi8658c->dataCallbackService.eraseCallback(id);
                self->qmi8658c->onTerminate();
                self->qmi8658c.reset();
                self->qmi8658c = nullptr;
                self->eventGroup = nullptr;
                self->onDataCallback = nullptr;
                self->int1Pin = GPIO_NUM_MAX;
                self->int2Pin = GPIO_NUM_MAX;
                vTaskDelete(nullptr);
            }};

            bool okToRun{true};
            while (okToRun) {
                xEventGroupWaitBits(
                    self->eventGroup,
                    DATA_AVAILABLE | SHUTDOWN_EVENT,
                    pdTRUE,
                    pdFALSE,
                    portMAX_DELAY);
                const uint8_t bitsSet = xEventGroupGetBits(self->eventGroup);

                if (bitsSet & SHUTDOWN_EVENT) {
                    okToRun = false;
                    continue;
                }
                if (bitsSet & DATA_AVAILABLE) {
                    self->qmi8658c->onSample();
                }
            }
        },
        "board_analitics",
        4096,
        this,
        5,
        nullptr);

    isRunning = true;
}
void MouseEventDetector::analyze(const Qmi8658C::DataPoint &data)
{
    ESP_LOGI(
        MouseEventDetector::TAG,
        "%05.2f, %05.2f, %05.2f, %05.2f, %05.2f, %05.2f, %05.2f, %05.2f, %05.2f",
        data.accelerometer.x,
        data.accelerometer.y,
        data.accelerometer.z,
        data.gyroscope.x,
        data.gyroscope.y,
        data.gyroscope.z,
        data.orientation.x,
        data.orientation.y,
        data.orientation.z);
}
} // namespace Hilton