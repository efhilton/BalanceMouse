#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <NotificationCallback.hpp>
#include <any>
#include <functional>
#include <map>

namespace Hilton {

template <typename T> class QueuedNotificationService {
public:
  QueuedNotificationService(const size_t queueSize, const char *taskName,
                            const uint32_t stackSize = 2048,
                            const UBaseType_t priority = tskIDLE_PRIORITY + 1)
      : queue_(xQueueCreate(queueSize, sizeof(T))), queueSize_(queueSize) {
    callbacksMutex_ = xSemaphoreCreateMutex();
    if (callbacksMutex_ == nullptr) {
      ESP_LOGE(TAG, "Failed to create mutex");
      return;
    }
    if (queue_ == nullptr) {
      ESP_LOGE(TAG, "Failed to create queue for notifications");
    }

    // Start the processing task
    xTaskCreate(&QueuedNotificationService::notificationTask, taskName,
                stackSize, this, priority, &taskHandle_);
  }

  ~QueuedNotificationService() {
    if (taskHandle_ != nullptr) {
      vTaskDelete(taskHandle_);
    }

    if (queue_ != nullptr) {
      vQueueDelete(queue_);
    }

    if (callbacksMutex_ != nullptr) {
      vSemaphoreDelete(callbacksMutex_);
    }
  }

  // Add a listener callback
  int addCallback(OnDataCallback<T> callback, std::any userData = std::any()) {
    if (xSemaphoreTake(callbacksMutex_, portMAX_DELAY) == pdTRUE) {
      const int id = next_id_;
      callbacks_[id] = CallbackInfo{callback, userData};
      next_id_++;
      xSemaphoreGive(callbacksMutex_);
      return id;
    } else {
      ESP_LOGE(TAG, "Failed to acquire mutex for adding callback");
      return -1; // Indicate failure
    }
  }

  // Remove a listener callback
  void eraseCallback(size_t id) {
    if (xSemaphoreTake(callbacksMutex_, portMAX_DELAY) == pdTRUE) {
      callbacks_.erase(id);
      xSemaphoreGive(callbacksMutex_);
    } else {
      ESP_LOGE(TAG, "Failed to acquire mutex for erasing callback");
    }
  }

  // Queue a notification to be processed in the listener thread
  void notify(const T &data) {
    if (queue_ == nullptr) {
      ESP_LOGE(TAG, "Queue is not initialized");
      return;
    }

    if (xQueueSendToBack(queue_, &data, portMAX_DELAY) != pdPASS) {
      ESP_LOGW(TAG, "Notification queue is full, dropping event");
    }
  }

private:
  static constexpr auto TAG{"QUEUED_NOTIFICATION_SERVICE"};
  SemaphoreHandle_t callbacksMutex_{nullptr};
  std::map<size_t, CallbackInfo<T>> callbacks_{};
  size_t next_id_{0};

  QueueHandle_t queue_{nullptr};
  TaskHandle_t taskHandle_{nullptr};
  size_t queueSize_;

  // Task to process queued notifications
  static void notificationTask(void *param) {
    auto *service = static_cast<QueuedNotificationService *>(param);

    T data;
    while (true) {
      // Wait for data on the queue
      if (xQueueReceive(service->queue_, &data, portMAX_DELAY) == pdPASS) {
        // Notify all registered callbacks
        if (xSemaphoreTake(service->callbacksMutex_, portMAX_DELAY) == pdTRUE) {
          for (const auto &[id, callbackInfo] : service->callbacks_) {
            try {
              if (callbackInfo.callback) {
                callbackInfo.callback(data, callbackInfo.userData);
              }
            } catch (const std::exception &e) {
              ESP_LOGE(TAG, "Exception in callback id=%d: %s ---> Ignoring", id,
                       e.what());
            } catch (...) {
              ESP_LOGE(TAG, "Unknown exception in callback id=%d ---> Ignoring",
                       id);
            }
          }
          xSemaphoreGive(service->callbacksMutex_);
        } else {
          ESP_LOGE(TAG, "Failed to acquire mutex for processing callbacks");
        }
      }
    }
  }
};

} // namespace Hilton