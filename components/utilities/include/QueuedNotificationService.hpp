#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <NotificationCallback.hpp>
#include <any>
#include <atomic>
#include <functional>
#include <map>
#include <set>
#include <string>

namespace Hilton {

/**
 * @brief A thread-safe notification service that queues and dispatches notifications to registered callbacks
 * 
 * This service allows registering multiple callbacks that can receive notifications based on a key.
 * Notifications are queued and processed asynchronously in a dedicated FreeRTOS task.
 * 
 * Template Parameters:
 * @tparam KeyDt Enum type used as key to identify different notification types
 * @tparam DataDt Type of the data passed in notifications (e.g. struct, int, string)
 * @tparam UserDt Type of the user data passed to callbacks (defaults to std::any)
 * 
 * Example usage:
 * @code
 * // Define notification types and data
 * enum class NotificationType { SensorData, SystemEvent };
 * struct SensorReading { float temperature; float humidity; };
 * 
 * // Create notification service
 * QueuedNotificationService<NotificationType, SensorReading> notifier(10, "NotifyTask");
 * 
 * // Add callback
 * auto callback = [](const SensorReading& data, const std::any& userData) {
 *     printf("Temp: %.1f, Humidity: %.1f\n", data.temperature, data.humidity);
 * };
 * uint32_t callbackId = notifier.addCallback(NotificationType::SensorData, callback);
 * 
 * // Send notification
 * SensorReading reading{25.5f, 60.0f};
 * notifier.notify(NotificationType::SensorData, reading);
 * 
 * // Remove callback when done
 * notifier.eraseCallback(callbackId);
 * @endcode
 */

template<
    typename KeyDt,
    typename DataDt,
    typename UserDt = std::any,
    typename = std::enable_if_t<std::is_enum_v<KeyDt>>>
class QueuedNotificationService
{
    struct QueueItem
    {
        KeyDt key;
        DataDt data;
    };
    std::vector<CallbackInfo<KeyDt, DataDt, UserDt>> callbacks;

public:
    /**
     * @brief Constructs a new Queued Notification Service
     * @param queueSize Maximum number of notifications that can be queued
     * @param taskName Name of the FreeRTOS task that processes notifications
     * @param stackSize Stack size for the notification processing task (default: 2048)
     * @param priority Priority of the notification processing task (default: tskIDLE_PRIORITY + 1)
     */
    QueuedNotificationService(
        const size_t queueSize,
        const char *taskName,
        const uint32_t stackSize = 2048,
        const UBaseType_t priority = tskIDLE_PRIORITY + 1)
        : callbacks()
        , queue(xQueueCreate(queueSize, sizeof(QueueItem)))
        , queueSize(queueSize)
    {
        callbacksMutex = xSemaphoreCreateMutex();
        if (callbacksMutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return;
        }
        if (queue == nullptr) {
            ESP_LOGE(TAG, "Failed to create queue for notifications");
        }

        // Start the processing task
        auto created = xTaskCreate(
            &QueuedNotificationService::notificationTask,
            taskName,
            stackSize,
            this,
            priority,
            &taskHandle);
        if (created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create notification task");
        }
    }

    /**
     * @brief Destructor that cleans up FreeRTOS resources
     */
    ~QueuedNotificationService()
    {
        if (taskHandle != nullptr) {
            vTaskDelete(taskHandle);
        }

        if (queue != nullptr) {
            vQueueDelete(queue);
        }

        if (callbacksMutex != nullptr) {
            vSemaphoreDelete(callbacksMutex);
        }
    }

    /**
     * @brief Adds a callback function for notifications
     * @param key The key to associate with this callback
     * @param callback Function to be called when notifications arrive
     * @param userData Optional user data to be passed to callback
     * @return Unique identifier for the callback, or -1 if registration failed
     */
    uint32_t addCallback(
        KeyDt key, OnDataCallback<KeyDt, DataDt, UserDt> callback, std::any userData = std::any())
    {
        static std::atomic<uint32_t> nextId{0};

        if (xSemaphoreTake(callbacksMutex, portMAX_DELAY) == pdTRUE) {
            const uint32_t id = nextId.fetch_add(1);
            callbacks.push_back(CallbackInfo{id, key, callback, userData});
            xSemaphoreGive(callbacksMutex);
            return id;
        }
        ESP_LOGE(TAG, "Failed to acquire mutex for adding callback");
        return -1; // Indicate failure
    }

    /**
     * @brief Removes a previously registered callback
     * @param id The unique identifier returned by addCallback
     */
    void eraseCallback(uint32_t id)
    {
        if (xSemaphoreTake(callbacksMutex, portMAX_DELAY) == pdTRUE) {
            callbacks.erase(
                std::remove_if(
                    callbacks.begin(),
                    callbacks.end(),
                    [id](const CallbackInfo<KeyDt, DataDt, UserDt> &info) { return info.id == id; }),
                callbacks.end());
            xSemaphoreGive(callbacksMutex);
        } else {
            ESP_LOGE(TAG, "Failed to acquire mutex for erasing callback");
        }
    }

    /**
     * @brief Sends a notification to all registered callbacks
     * @param key The key identifying which callbacks should receive the notification
     * @param data The data to be passed to the callbacks
     */
    void notify(const KeyDt key, const DataDt &data)
    {
        if (queue == nullptr) {
            ESP_LOGE(TAG, "Queue is not initialized");
            return;
        }
        QueueItem packet{key, data};
        if (xQueueSendToBack(queue, &packet, portMAX_DELAY) != pdPASS) {
            ESP_LOGW(TAG, "Notification queue is full, dropping event");
        }
    }

private:
    static constexpr auto TAG{"QUEUED_NOTIFICATION_SERVICE"};
    SemaphoreHandle_t callbacksMutex{nullptr};

    QueueHandle_t queue{nullptr};
    TaskHandle_t taskHandle{nullptr};
    size_t queueSize;

    // Task to process queued notifications
    static void notificationTask(void *param)
    {
        auto *service = static_cast<QueuedNotificationService *>(param);

        QueueItem packet;
        while (true) {
            // Wait for data on the queue
            if (xQueueReceive(service->queue, &packet, portMAX_DELAY) == pdPASS) {
                // Notify all registered callbacks
                if (xSemaphoreTake(service->callbacksMutex, portMAX_DELAY) == pdTRUE) {
                    for (const auto &[id, key, callback, userData] : service->callbacks) {
                        try {
                            if (key != packet.key) {
                                continue;
                            }
                            callback(key, packet.data, userData);
                        } catch (const std::exception &e) {
                            ESP_LOGE(
                                TAG, "Exception in callback id=%d: %s ---> Ignoring", id, e.what());
                        } catch (...) {
                            ESP_LOGE(TAG, "Unknown exception in callback id=%d ---> Ignoring", id);
                        }
                    }
                    xSemaphoreGive(service->callbacksMutex);
                } else {
                    ESP_LOGE(TAG, "Failed to acquire mutex for processing callbacks");
                }
            } else {
                ESP_LOGE(
                    TAG, "Failed to receive data from queue. This should never happen. Aborting.");
                break;
            }
        }
    }
};

} // namespace Hilton