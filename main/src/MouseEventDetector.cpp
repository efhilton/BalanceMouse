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
#include <algorithm>

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
bool isButtonIdle(const MouseButtonStateMachine &button)
{
    return button.getState() == MouseButtonStateMachine::State::Idle;
}

ChainOfResponsibility<Qmi8658C::DataPoint, MouseEventDetector::TrackingContext>::Callback clickHandler{
    [](const Qmi8658C::DataPoint &data, MouseEventDetector::TrackingContext &context) -> bool {
        constexpr auto X_PRESSED_THRESHOLD{8.0f}; // degrees
        constexpr auto X_RELEASED_THRESHOLD(5.0f); // degrees
        constexpr auto X_DOT_THRESHOLD{25.0f};     // degrees per second
        constexpr auto Y_PRESSED_THRESHOLD{8.0f};  // degrees
        constexpr auto Y_RELEASED_THRESHOLD{5.0f}; // degrees
        constexpr auto Y_DOT_THRESHOLD{25.0f};     // degrees per second
        constexpr auto Z_START_THRESHOLD{10.0f};   // degrees
        constexpr auto Z_STOP_THRESHOLD{1.0f};     // degrees
        constexpr auto X_MOUSE_THRESHOLD{5.0f};    // degrees
        constexpr auto X_MOUSE_SCALE{6.0f};
        constexpr auto Y_MOUSE_THRESHOLD{5.0f}; // degrees
        constexpr auto Y_MOUSE_SCALE{-8.0f};

        const float deltaX = data.tilt.x - context.deltaXFilter.getFilteredValue();
        const float deltaY = data.tilt.y - context.deltaYFilter.getFilteredValue();
        const float deltaZ = data.orientation.z - context.deltaZFilter.getFilteredValue();
        const bool isLeftButtonPressed = isButtonPressed(context.leftButton);
        const bool isMiddleButtonPressed = isButtonPressed(context.middleButton);
        const bool isRightButtonPressed = isButtonPressed(context.rightButton);
        const bool isLeftButtonIdle = isButtonIdle(context.leftButton);
        const bool isMiddleButtonIdle = isButtonIdle(context.middleButton);
        const bool isRightButtonIdle = isButtonIdle(context.rightButton);
        const bool isXButtonActionSpeed = std::abs(deltaX) > X_DOT_THRESHOLD;
        const bool isYButtonActionSpeed = std::abs(deltaY) > Y_DOT_THRESHOLD;

        if (isXButtonActionSpeed || isYButtonActionSpeed) {
            if (isXButtonActionSpeed && isLeftButtonPressed && deltaX > -X_RELEASED_THRESHOLD) {
                context.leftButton.handleEvent(MouseButtonStateMachine::Event::MouseUp);
            } else if (isXButtonActionSpeed && isRightButtonPressed && deltaX < X_RELEASED_THRESHOLD) {
                context.rightButton.handleEvent(MouseButtonStateMachine::Event::MouseUp);
            } else if (isYButtonActionSpeed && isMiddleButtonPressed && deltaY > -Y_RELEASED_THRESHOLD) {
                context.middleButton.handleEvent(MouseButtonStateMachine::Event::MouseUp);
            } else if (isLeftButtonIdle && isRightButtonIdle && isMiddleButtonIdle) {
                if (isXButtonActionSpeed && deltaX > X_PRESSED_THRESHOLD) {
                    context.rightButton.handleEvent(MouseButtonStateMachine::Event::MouseDown);
                } else if (isXButtonActionSpeed && deltaX < -X_PRESSED_THRESHOLD) {
                    context.leftButton.handleEvent(MouseButtonStateMachine::Event::MouseDown);
                } else if (isYButtonActionSpeed && deltaY < -Y_PRESSED_THRESHOLD) {
                    context.middleButton.handleEvent(MouseButtonStateMachine::Event::MouseDown);
                }
            }
            context.event.deltaX = 0;
            context.event.deltaY = 0;
        } else if (std::abs(deltaX) > X_MOUSE_THRESHOLD || std::abs(deltaY) > Y_MOUSE_THRESHOLD) {
            const int8_t x = static_cast<int8_t>(
                std::round(std::clamp(X_MOUSE_SCALE * deltaX, -127.0f, 127.0f)));
            const int8_t y = static_cast<int8_t>(
                std::round(std::clamp(Y_MOUSE_SCALE * deltaY, -127.0f, 127.0f)));
            context.event.deltaX = x;
            context.event.deltaY = y;
        } else {
            context.event.deltaX = 0;
            context.event.deltaY = 0;
        }

        if (std::abs(deltaZ) > Z_START_THRESHOLD && !context.isWheelScrolling) {
            context.isWheelScrolling = true;
        } else if (std::abs(deltaZ) < Z_STOP_THRESHOLD && context.isWheelScrolling) {
            context.isWheelScrolling = false;
        }

        if (context.isWheelScrolling) {
            float scaledWheel = 2.5f * deltaZ;
            scaledWheel = std::clamp(scaledWheel, -127.0f, 127.0f);
            context.event.wheel = static_cast<int8_t>(std::round(scaledWheel));
        } else {
            context.event.wheel = 0;
        }

        context.deltaXFilter.filter(data.tilt.x);
        context.deltaYFilter.filter(data.tilt.y);
        context.deltaZFilter.filter(data.orientation.z);
        return false;
    }};

ChainOfResponsibility<Qmi8658C::DataPoint, MouseEventDetector::TrackingContext>::Callback
    cleanupEventsHandler{
        [](const Qmi8658C::DataPoint &data, MouseEventDetector::TrackingContext &context) -> bool {
            bool isLeftPressed = isButtonPressed(context.leftButton);
            bool isMiddlePressed = isButtonPressed(context.middleButton);
            bool isRightPressed = isButtonPressed(context.rightButton);
            context.event.buttons
                = (isLeftPressed ? MouseEventDetector::MouseButtonMask::LeftButton : 0)
                  | (isMiddlePressed ? MouseEventDetector::MouseButtonMask::MiddleButton : 0)
                  | (isRightPressed ? MouseEventDetector::MouseButtonMask::RightButton : 0);

            return false;
        }};

ChainOfResponsibility<Qmi8658C::DataPoint, MouseEventDetector::TrackingContext>::Callback printStats{
    [](const Qmi8658C::DataPoint &data, const MouseEventDetector::TrackingContext &context) -> bool {
        printf(
            "(x,y):(%4d,%4d), W: %4d, L:%-15s, M:%-15s, R:%-15s\n",
            context.event.deltaX,
            context.event.deltaY,
            context.event.wheel,
            context.leftButton.getStateName().c_str(),
            context.middleButton.getStateName().c_str(),
            context.rightButton.getStateName().c_str());
        return true;
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

    // chainOfResponsibility.addHandler(movementHandler);
    chainOfResponsibility.addHandler(clickHandler);
    chainOfResponsibility.addHandler(cleanupEventsHandler);
    chainOfResponsibility.addHandler(printStats);
    onDataCallback = [this](const Qmi8658C::DataPoint &data) {
        getChainOfResponsibility().handle(data, trackingContext);
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