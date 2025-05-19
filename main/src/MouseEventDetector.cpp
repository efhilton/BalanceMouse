#include "MouseEventDetector.hpp"
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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(self->eventGroup, DATA_AVAILABLE, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

bool isButtonPressed(const MouseButtonStateMachine &button)
{
    return button.getState() == MouseButtonStateMachine::State::Pressed;
}

ChainOfResponsibility<Qmi8658C::DataPoint, MouseEventDetector::TrackingContext>::Callback
    movementHandler{
        [](const Qmi8658C::DataPoint &data, MouseEventDetector::TrackingContext &event) -> bool {
            event.event.deltaX = static_cast<int8_t>(data.gyroscope.x);
            event.event.deltaY = static_cast<int8_t>(data.gyroscope.y);
            return false;
        }};

ChainOfResponsibility<Qmi8658C::DataPoint, MouseEventDetector::TrackingContext>::Callback
    clickHandler{

        [](const Qmi8658C::DataPoint &data, MouseEventDetector::TrackingContext &event) -> bool {
            constexpr auto Z_PRESSED_THRESHOLD{10}; // degrees
            constexpr auto Z_RELEASED_THRESHOLD(2); // degrees
            constexpr auto Y_PRESSED_THRESHOLD{10}; // degrees
            constexpr auto Y_RELEASED_THRESHOLD{2}; // degrees

            const float deltaZ = data.orientation.z - event.deltaZFilter.getFilteredValue();
            const float deltaY = data.orientation.y - event.deltaYFilter.getFilteredValue();
            // printf(
            //     "instant: %f, filtered: %f, deltaY: %f\n",
            //     data.orientation.y,
            //     event.deltaYFilter.getFilteredValue(),
            //     deltaY);
            if (deltaZ > -Z_RELEASED_THRESHOLD && isButtonPressed(event.leftButton)) {
                event.leftButton.handleEvent(
                    MouseButtonStateMachine::Event::MouseUp, data.gyroscope.x, data.gyroscope.y);

            } else if (deltaZ < -Z_PRESSED_THRESHOLD) {
                event.leftButton.handleEvent(
                    MouseButtonStateMachine::Event::MouseDown, data.gyroscope.x, data.gyroscope.y);
            }
            if (deltaZ < Z_RELEASED_THRESHOLD && isButtonPressed(event.rightButton)) {
                event.rightButton.handleEvent(
                    MouseButtonStateMachine::Event::MouseUp, data.gyroscope.x, data.gyroscope.y);
            } else if (deltaZ > Z_PRESSED_THRESHOLD) {
                event.rightButton.handleEvent(
                    MouseButtonStateMachine::Event::MouseDown, data.gyroscope.x, data.gyroscope.y);
            }
            if (deltaY > -Y_RELEASED_THRESHOLD && isButtonPressed(event.middleButton)) {
                event.middleButton.handleEvent(
                    MouseButtonStateMachine::Event::MouseUp, data.gyroscope.x, data.gyroscope.y);
            } else if (deltaY < -Y_PRESSED_THRESHOLD) {
                event.middleButton.handleEvent(
                    MouseButtonStateMachine::Event::MouseDown, data.gyroscope.x, data.gyroscope.y);
            }
            event.deltaZFilter.filter(data.orientation.z);
            event.deltaYFilter.filter(data.orientation.y);
            return false;
        }};

MouseEventDetector::MouseEventDetector(
    const std::shared_ptr<Qmi8658C> &qmi8658C, const gpio_num_t int1Pin, const gpio_num_t int2Pin)
    : qmi8658c(qmi8658C)
    , int1Pin(int1Pin)
    , int2Pin(int2Pin)
    , eventGroup(xEventGroupCreate())
{
    gpio_reset_pin(int1Pin);
    gpio_reset_pin(int2Pin);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << int1Pin) | (1ULL << int2Pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(int1Pin, gpioIsrHandler, this);
    gpio_isr_handler_add(int2Pin, gpioIsrHandler, this);

    chainOfResponsibility.addHandler(movementHandler);
    chainOfResponsibility.addHandler(clickHandler);
    onDataCallback = [this](const Qmi8658C::DataPoint &data) {
        getChainOfResponsibility().handle(data, trackingContext);
        // printf(
        //     "%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f\n",
        //     data.accelerometer.x,
        //     data.accelerometer.y,
        //     data.accelerometer.z,
        //     data.gyroscope.x,
        //     data.gyroscope.y,
        //     data.gyroscope.z,
        //     data.orientation.x,
        //     data.orientation.y,
        //     data.orientation.z);
    };
}

void MouseEventDetector::run()
{
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
                    pdMS_TO_TICKS(34));
                const uint8_t bitsSet = xEventGroupGetBits(self->eventGroup);

                if (bitsSet & SHUTDOWN_EVENT) {
                    okToRun = false;
                    continue;
                }
                if (bitsSet & DATA_AVAILABLE) {
                    ESP_LOGE(self->TAG, "DATA_AVAILABLE");
                }
                self->qmi8658c->onSample();
            }
        },
        "board_analitics",
        4096,
        this,
        5,
        nullptr);

    isRunning = true;
}

} // namespace Hilton