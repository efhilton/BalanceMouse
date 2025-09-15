#include "MouseEventDetector.hpp"
#include <Qmi8658C.hpp>
#include <memory>
#include <string>

#include "Finalizer.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "soc/gpio_num.h"
#include <algorithm>
#include <atomic>

namespace Hilton {
static constexpr auto DATA_AVAILABLE{BIT0};
static constexpr auto DO_SHUTDOWN{BIT1};
static constexpr auto IS_RUNNING{BIT2};
static constexpr auto IS_SHUTDOWN{BIT3};
static constexpr auto TAG{"MOUSE_EVENT_DETECTOR"};
std::atomic eventCounter{0UL};

bool isButtonPressed(const MouseButtonStateMachine &button) {
  return button.getState() == MouseButtonStateMachine::State::Pressed;
}
bool isButtonIdle(const MouseButtonStateMachine &button) {
  return button.getState() == MouseButtonStateMachine::State::Idle;
}

ChainOfResponsibility<Qmi8658C::DataPoint,
                      MouseEventDetector::TrackingContext>::Callback
    clickHandler{[](const Qmi8658C::DataPoint &data,
                    MouseEventDetector::TrackingContext &context) -> bool {
      constexpr auto X_PRESSED_THRESHOLD{10.0f}; // degrees
      constexpr auto X_RELEASED_THRESHOLD(2.0f); // degrees
      constexpr auto X_DOT_THRESHOLD{55.0f};     // degrees per second
      constexpr auto Y_PRESSED_THRESHOLD{10.0f}; // degrees
      constexpr auto Y_RELEASED_THRESHOLD{2.0f}; // degrees
      constexpr auto Y_DOT_THRESHOLD{55.0f};     // degrees per second
      constexpr auto Z_START_THRESHOLD{10.0f};   // degrees
      constexpr auto Z_STOP_THRESHOLD{1.0f};     // degrees
      constexpr auto X_MOUSE_THRESHOLD{2.0f};    // degrees
      constexpr auto X_MOUSE_SCALE{7.0f};        // ints/degree
      constexpr auto Y_MOUSE_THRESHOLD{2.0f};    // degrees
      constexpr auto Y_MOUSE_SCALE{-8.0f};       // ints/degree

      const float deltaX =
          data.tilt.x - context.deltaXFilter.getFilteredValue();
      const float deltaY =
          data.tilt.y - context.deltaYFilter.getFilteredValue();
      const float deltaZ =
          data.orientation.z - context.deltaZFilter.getFilteredValue();
      const bool isLeftButtonPressed = isButtonPressed(context.leftButton);
      const bool isMiddleButtonPressed = isButtonPressed(context.middleButton);
      const bool isRightButtonPressed = isButtonPressed(context.rightButton);
      const bool isLeftButtonIdle = isButtonIdle(context.leftButton);
      const bool isMiddleButtonIdle = isButtonIdle(context.middleButton);
      const bool isRightButtonIdle = isButtonIdle(context.rightButton);
      const bool isXButtonActionSpeed =
          std::abs(data.gyroscope.x) > X_DOT_THRESHOLD;
      const bool isYButtonActionSpeed =
          std::abs(data.gyroscope.y) > Y_DOT_THRESHOLD;

      if (std::abs(deltaZ) > Z_START_THRESHOLD && !context.isWheelScrolling) {
        context.isWheelScrolling = true;
      } else if (std::abs(deltaZ) < Z_STOP_THRESHOLD &&
                 context.isWheelScrolling) {
        context.isWheelScrolling = false;
      }

      if (!context.isWheelScrolling &&
          (isXButtonActionSpeed || isYButtonActionSpeed)) {
        if (isXButtonActionSpeed && isLeftButtonPressed &&
            deltaX > -X_RELEASED_THRESHOLD) {
          context.leftButton.handleEvent(
              MouseButtonStateMachine::Event::MouseUp);
        } else if (isXButtonActionSpeed && isRightButtonPressed &&
                   deltaX < X_RELEASED_THRESHOLD) {
          context.rightButton.handleEvent(
              MouseButtonStateMachine::Event::MouseUp);
        } else if (isYButtonActionSpeed && isMiddleButtonPressed &&
                   deltaY > -Y_RELEASED_THRESHOLD) {
          context.middleButton.handleEvent(
              MouseButtonStateMachine::Event::MouseUp);
        } else if (isLeftButtonIdle && isRightButtonIdle &&
                   isMiddleButtonIdle) {
          if (isXButtonActionSpeed && deltaX > X_PRESSED_THRESHOLD) {
            context.rightButton.handleEvent(
                MouseButtonStateMachine::Event::MouseDown);
          } else if (isXButtonActionSpeed && deltaX < -X_PRESSED_THRESHOLD) {
            context.leftButton.handleEvent(
                MouseButtonStateMachine::Event::MouseDown);
          } else if (isYButtonActionSpeed && deltaY < -Y_PRESSED_THRESHOLD) {
            context.middleButton.handleEvent(
                MouseButtonStateMachine::Event::MouseDown);
          }
        }
        context.event.deltaX = 0;
        context.event.deltaY = 0;
      } else if (!context.isWheelScrolling &&
                 (std::abs(deltaX) > X_MOUSE_THRESHOLD ||
                  std::abs(deltaY) > Y_MOUSE_THRESHOLD)) {
        const int8_t x = static_cast<int8_t>(
            std::round(std::clamp(X_MOUSE_SCALE * deltaX, -127.0f, 127.0f)));
        const int8_t y = static_cast<int8_t>(
            std::round(std::clamp(Y_MOUSE_SCALE * deltaY, -127.0f, 127.0f)));
        context.event.deltaX = x;
        context.event.deltaY = y;
        context.event.wheel = 0;
      } else if (context.isWheelScrolling) {
        float scaledWheel = 2.5f * deltaZ;
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

ChainOfResponsibility<Qmi8658C::DataPoint,
                      MouseEventDetector::TrackingContext>::Callback
    cleanupEventsHandler{
        [](const Qmi8658C::DataPoint &data,
           MouseEventDetector::TrackingContext &context) -> bool {
          bool isLeftPressed = isButtonPressed(context.leftButton);
          bool isMiddlePressed = isButtonPressed(context.middleButton);
          bool isRightPressed = isButtonPressed(context.rightButton);
          context.event.buttons =
              (isLeftPressed ? MouseButtonMask::LeftButton : 0) |
              (isMiddlePressed ? MouseButtonMask::MiddleButton : 0) |
              (isRightPressed ? MouseButtonMask::RightButton : 0);

          return false;
        }};

ChainOfResponsibility<Qmi8658C::DataPoint,
                      MouseEventDetector::TrackingContext>::Callback printStats{
    [](const Qmi8658C::DataPoint &data,
       const MouseEventDetector::TrackingContext &context) -> bool {
      printf("(X,Y):(%4d,%4d), W: %4d, L:%-15s, M:%-15s, R:%-15s\n",
             context.event.deltaX, context.event.deltaY, context.event.wheel,
             context.leftButton.getStateName().c_str(),
             context.middleButton.getStateName().c_str(),
             context.rightButton.getStateName().c_str());
      return true;
    }};

static void IRAM_ATTR gpioIsrHandler(void *p) {
  eventCounter.fetch_add(1);
  auto *self = static_cast<MouseEventDetector *>(p);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(self->dataEventGroup, DATA_AVAILABLE,
                            &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

MouseEventDetector::MouseEventDetector(
    const std::shared_ptr<Qmi8658C> &qmi8658C, const gpio_num_t int1Pin,
    const gpio_num_t int2Pin)
    : eventCallbackService(10, "EventsCallbackTask"), qmi8658c(qmi8658C),
      int1Pin(int1Pin), int2Pin(int2Pin), dataEventGroup(xEventGroupCreate()) {
  assert(qmi8658c);
  assert(dataEventGroup);

  gpio_reset_pin(int1Pin);
  gpio_reset_pin(int2Pin);

  gpio_config_t io_conf = {};
  io_conf.intr_type =
      GPIO_INTR_ANYEDGE; // Use GPIO_INTR_POSEDGE or GPIO_INTR_NEGEDGE if only
                         // one edge is needed
  io_conf.pin_bit_mask = (1ULL << int1Pin) | (1ULL << int2Pin);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en =
      GPIO_PULLUP_ENABLE; // Ensure this matches your hardware setup
  io_conf.pull_down_en =
      GPIO_PULLDOWN_DISABLE; // Explicitly disable pull-down resistors

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE("GPIO", "Failed to configure GPIOs: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI("GPIO", "GPIOs configured: INT1=%d, INT2=%d", int1Pin, int2Pin);
  }

  // chainOfResponsibility.addHandler(movementHandler);
  chainOfResponsibility.addHandler(clickHandler);
  chainOfResponsibility.addHandler(cleanupEventsHandler);
  chainOfResponsibility.addHandler(printStats);

  gpio_install_isr_service(0);
  ret = gpio_isr_handler_add(int1Pin, gpioIsrHandler, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add ISR handler for INT1: %s",
             esp_err_to_name(ret));
  }

  ret = gpio_isr_handler_add(int2Pin, gpioIsrHandler, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add ISR handler for INT2: %s",
             esp_err_to_name(ret));
  }
}

void MouseEventDetector::run() {
  xEventGroupClearBits(dataEventGroup, IS_RUNNING | IS_SHUTDOWN | DO_SHUTDOWN);

  // Create a task for handling sensor data
  xTaskCreate(
      [](void *arg) {
        auto *self = static_cast<MouseEventDetector *>(arg);

        Finalizer f{[self] {
          ESP_LOGI(TAG, "Stopping data processing task");
          xEventGroupClearBits(self->dataEventGroup, IS_RUNNING);
          xEventGroupSetBits(self->dataEventGroup, IS_SHUTDOWN);
          vTaskDelete(nullptr);
        }};

        xEventGroupClearBits(self->dataEventGroup, IS_SHUTDOWN);
        xEventGroupSetBits(self->dataEventGroup, IS_RUNNING);
        Qmi8658C::DataPoint point{};
        ESP_LOGI(TAG, "Starting data processing task");
        while (true) {
          auto bits = xEventGroupWaitBits(self->dataEventGroup,
                                          DATA_AVAILABLE | DO_SHUTDOWN, pdTRUE,
                                          false, pdMS_TO_TICKS(10));
          if ((bits & DO_SHUTDOWN) == DO_SHUTDOWN) {
            break;
          }
          if ((bits & DATA_AVAILABLE) == DATA_AVAILABLE) {
            ESP_LOGI(TAG, "New data available");
            self->qmi8658c->onSample(point);
            self->chainOfResponsibility.handle(point, self->trackingContext);
          } else {
            self->qmi8658c->onSample(point);
            self->chainOfResponsibility.handle(point, self->trackingContext);
          }
        }
      },
      "board_analitics", 4096, this, 5, nullptr);
  const auto bits = xEventGroupWaitBits(dataEventGroup, IS_RUNNING, pdTRUE,
                                        false, pdMS_TO_TICKS(10000));
  ESP_LOGI(TAG, "Mouse detector is %s",
           (bits & IS_RUNNING) == IS_RUNNING ? "running" : "NOT running");
}

MouseEventDetector::~MouseEventDetector() {
  xEventGroupSetBits(dataEventGroup, DO_SHUTDOWN);
  xEventGroupWaitBits(dataEventGroup, IS_SHUTDOWN, pdTRUE, false,
                      portMAX_DELAY);
  gpio_isr_handler_remove(int1Pin);
  gpio_isr_handler_remove(int2Pin);
  vEventGroupDelete(dataEventGroup);
  gpio_uninstall_isr_service();
}

} // namespace Hilton