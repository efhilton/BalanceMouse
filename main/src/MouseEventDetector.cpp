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
#include <atomic>

using MouseHandler = Hilton::ChainOfResponsibility<
    Hilton::Qmi8658C::DataPoint,
    Hilton::MouseEventDetector::TrackingContext>::CorHandler;

namespace Hilton {
static constexpr EventBits_t DATA_AVAILABLE{BIT0};
static constexpr EventBits_t DO_SHUTDOWN{BIT1};
static constexpr EventBits_t IS_RUNNING{BIT2};
static constexpr EventBits_t IS_SHUTDOWN{BIT3};

std::atomic eventCounter{0UL};

bool isButtonPressed(const MouseButtonStateMachine &button)
{
    return button.getState() == MouseButtonStateMachine::State::Pressed;
}
bool isButtonIdle(const MouseButtonStateMachine &button)
{
    return button.getState() == MouseButtonStateMachine::State::Idle;
}

MouseHandler clickHandler{
    [](const Qmi8658C::DataPoint &data, MouseEventDetector::TrackingContext &context) -> bool {
        constexpr auto X_PRESSED_THRESHOLD{10.0f}; // degrees
        constexpr auto X_RELEASED_THRESHOLD(2.0f); // degrees
        constexpr auto X_DOT_THRESHOLD{55.0f};     // degrees per second
        constexpr auto Y_PRESSED_THRESHOLD{10.0f}; // degrees
        constexpr auto Y_RELEASED_THRESHOLD{2.0f}; // degrees
        constexpr auto Y_DOT_THRESHOLD{55.0f};     // degrees per second
        constexpr auto Z_START_THRESHOLD{10.0f};   // degrees
        constexpr auto Z_STOP_THRESHOLD{8.0f};     // degrees (hysteresis)
        constexpr auto X_MOUSE_THRESHOLD{2.0f};    // degrees
        constexpr auto X_MOUSE_SCALE{7.0f};        // ints/degree
        constexpr auto Y_MOUSE_THRESHOLD{2.0f};    // degrees
        constexpr auto Y_MOUSE_SCALE{-8.0f};       // ints/degree

        const float deltaX = data.tilt.x - context.deltaXFilter.getFilteredValue();
        const float deltaY = data.tilt.y - context.deltaYFilter.getFilteredValue();
        // For scroll detection, use absolute orientation angle (Z) rather than high-pass delta
        const float absOrientZ = std::abs(data.orientation.z);
        const bool isLeftButtonPressed = isButtonPressed(context.leftButton);
        const bool isMiddleButtonPressed = isButtonPressed(context.middleButton);
        const bool isRightButtonPressed = isButtonPressed(context.rightButton);
        const bool isLeftButtonIdle = isButtonIdle(context.leftButton);
        const bool isMiddleButtonIdle = isButtonIdle(context.middleButton);
        const bool isRightButtonIdle = isButtonIdle(context.rightButton);
        const bool isXButtonActionSpeed = std::abs(data.gyroscope.x) > X_DOT_THRESHOLD;
        const bool isYButtonActionSpeed = std::abs(data.gyroscope.y) > Y_DOT_THRESHOLD;

        if (absOrientZ > Z_START_THRESHOLD && !context.isWheelScrolling) {
            context.isWheelScrolling = true;
        } else if (absOrientZ < Z_STOP_THRESHOLD && context.isWheelScrolling) {
            context.isWheelScrolling = false;
        }

        if (!context.isWheelScrolling && (isXButtonActionSpeed || isYButtonActionSpeed)) {
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
        } else if (
            !context.isWheelScrolling
            && (std::abs(deltaX) > X_MOUSE_THRESHOLD || std::abs(deltaY) > Y_MOUSE_THRESHOLD)) {
            const int8_t x = static_cast<int8_t>(
                std::round(std::clamp(X_MOUSE_SCALE * deltaX, -127.0f, 127.0f)));
            const int8_t y = static_cast<int8_t>(
                std::round(std::clamp(Y_MOUSE_SCALE * deltaY, -127.0f, 127.0f)));
            context.event.deltaX = x;
            context.event.deltaY = y;
            context.event.wheel = 0;
        } else if (context.isWheelScrolling) {
            // Scale wheel proportionally to absolute orientation angle around Z, clamped to 90°
            float clampedAngle = std::clamp(data.orientation.z, -90.0f, 90.0f);
            float scaledWheel = (127.0f / 90.0f) * clampedAngle;
            scaledWheel = std::clamp(scaledWheel, -127.0f, 127.0f);
            context.event.wheel = static_cast<int8_t>(std::round(scaledWheel));
            context.event.deltaX = 0;
            context.event.deltaY = 0;
        } else {
            context.event.deltaX = 0;
            context.event.deltaY = 0;
            context.event.wheel = 0;
        }

        context.deltaXFilter.filter(data.tilt.x);
        context.deltaYFilter.filter(data.tilt.y);
        context.deltaZFilter.filter(data.orientation.z);
        return false;
    }};

MouseHandler mouseClickBuilder{
    [](const Qmi8658C::DataPoint &data, MouseEventDetector::TrackingContext &context) -> bool {
        const bool isLeftPressed = isButtonPressed(context.leftButton);
        const bool isMiddlePressed = isButtonPressed(context.middleButton);
        const bool isRightPressed = isButtonPressed(context.rightButton);
        context.event.buttons = (isLeftPressed ? LeftButton : None)
                                | (isMiddlePressed ? MiddleButton : None)
                                | (isRightPressed ? RightButton : None);

        return false;
    }};

MouseHandler printRaw{
    [](const Qmi8658C::DataPoint &data, const MouseEventDetector::TrackingContext &context) -> bool {
        printf(
            "Accel (%6.2f %6.2f %6.2f), Gyro (%6.2f, %6.2f, %6.2f)\n",
            data.accelerometer.x,
            data.accelerometer.y,
            data.accelerometer.z,
            data.gyroscope.x,
            data.gyroscope.y,
            data.gyroscope.z);
        return false;
    }};

MouseHandler printDetectedEvents{
    [](const Qmi8658C::DataPoint &data, const MouseEventDetector::TrackingContext &context) -> bool {
        printf(
            "(X,Y):(%4d,%4d), W: %4d, L:%-15s, M:%-15s, R:%-15s\n",
            context.event.deltaX,
            context.event.deltaY,
            context.event.wheel,
            context.leftButton.getStateName().c_str(),
            context.middleButton.getStateName().c_str(),
            context.rightButton.getStateName().c_str());
        return false;
    }};

static void IRAM_ATTR gpioIsrHandler(void *p)
{
    auto *self = static_cast<MouseEventDetector *>(p);
    // Only signal when an INT line indicates data-ready (active low assumed)
    int level1 = gpio_get_level(self->int1Pin);
    int level2 = gpio_get_level(self->int2Pin);
    if (level1 == 0 || level2 == 0) {
        eventCounter.fetch_add(1);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(self->dataEventGroup, DATA_AVAILABLE, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

MouseEventDetector::MouseEventDetector(
    const std::shared_ptr<Qmi8658C> &qmi8658C, const gpio_num_t int1Pin, const gpio_num_t int2Pin)
    : eventCallbackService(10, "EventsCallbackTask")
    , qmi8658c(qmi8658C)
    , int1Pin(int1Pin)
    , int2Pin(int2Pin)
    , dataEventGroup(xEventGroupCreate())
{
    assert(qmi8658c);
    assert(dataEventGroup);

    gpio_reset_pin(int1Pin);
    gpio_reset_pin(int2Pin);

    gpio_config_t io_conf = {};
    // Configure interrupt to trigger only on the data-ready assertion edge.
    // QMI8658C INT can be configured as active-low (recommended); use falling edge with pull-up.
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
                                           // one edge is needed
    io_conf.pin_bit_mask = (1ULL << int1Pin) | (1ULL << int2Pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // Ensure this matches your hardware setup
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Explicitly disable pull-down resistors

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("GPIO", "Failed to configure GPIOs: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI("GPIO", "GPIOs configured: INT1=%d, INT2=%d", int1Pin, int2Pin);
    }

    // Register handlers: functional first, then optional debug printers
    // chainOfResponsibility.addHandler(clickHandler);
    // chainOfResponsibility.addHandler(mouseClickBuilder);
    // chainOfResponsibility.addHandler(printDetectedEvents); // non-blocking debug
    chainOfResponsibility.addHandler(printRaw); // verbose raw data if needed
}

void MouseEventDetector::run()
{
    xEventGroupClearBits(dataEventGroup, IS_RUNNING | IS_SHUTDOWN | DO_SHUTDOWN);

    // Create a task for handling sensor data
    xTaskCreate(
        [](void *arg) {
            auto *self = static_cast<MouseEventDetector *>(arg);

            const Finalizer f{[self] {
                ESP_LOGI(TAG, "Stopping data processing task");
                gpio_isr_handler_remove(self->int1Pin);
                gpio_isr_handler_remove(self->int2Pin);
                xEventGroupClearBits(self->dataEventGroup, IS_RUNNING);
                xEventGroupSetBits(self->dataEventGroup, IS_SHUTDOWN);
                vTaskDelete(nullptr);
            }};

            xEventGroupClearBits(self->dataEventGroup, IS_SHUTDOWN);
            xEventGroupSetBits(self->dataEventGroup, IS_RUNNING);
            Qmi8658C::DataPoint point{};

            TrackingContext trackingContext{};

            gpio_install_isr_service(0);
            esp_err_t ret = gpio_isr_handler_add(self->int1Pin, gpioIsrHandler, self);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add ISR handler for INT1: %s", esp_err_to_name(ret));
            }

            ret = gpio_isr_handler_add(self->int2Pin, gpioIsrHandler, self);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add ISR handler for INT2: %s", esp_err_to_name(ret));
            }
            ESP_LOGI(TAG, "Starting data processing task");
            while (true) {
                auto bits = xEventGroupWaitBits(
                    self->dataEventGroup,
                    DATA_AVAILABLE | DO_SHUTDOWN,
                    pdTRUE,
                    false,
                    portMAX_DELAY);
                if ((bits & DO_SHUTDOWN) == DO_SHUTDOWN) {
                    break;
                }
                if ((bits & DATA_AVAILABLE) == DATA_AVAILABLE) {
                    if (eventCounter.load() > 0) {
                        ESP_LOGD(TAG, "Data-ready event(s): %lu", eventCounter.load());
                    }
                    // Read sample only when data-ready triggered
                    if (self->qmi8658c->onSample(point) == ESP_OK) {
                        self->chainOfResponsibility.handle(point, trackingContext);
                    } else {
                        ESP_LOGE(TAG, "Failed to process data (%s)", esp_err_to_name(ret));
                    }
                }
                // If not DATA_AVAILABLE, just loop back and wait again (portMAX_DELAY).
            }
        },
        "board_analitics",
        4096,
        this,
        5,
        nullptr);
    const auto bits
        = xEventGroupWaitBits(dataEventGroup, IS_RUNNING, pdTRUE, false, pdMS_TO_TICKS(10000));
    ESP_LOGI(
        TAG, "Mouse detector is %s", (bits & IS_RUNNING) == IS_RUNNING ? "running" : "NOT running");
}

MouseEventDetector::~MouseEventDetector()
{
    xEventGroupSetBits(dataEventGroup, DO_SHUTDOWN);
    xEventGroupWaitBits(dataEventGroup, IS_SHUTDOWN, pdTRUE, false, portMAX_DELAY);

    vEventGroupDelete(dataEventGroup);
    gpio_uninstall_isr_service();
}

} // namespace Hilton
