#pragma once
#include <algorithm>
#include <any>
#include <atomic>
#include <esp_log.h>
#include <exception>
#include <functional>
#include <NotificationCallback.hpp>
#include <vector>

// FreeRTOS-friendly mutex implementation
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace Hilton {
    class FreeRTOSMutex {
    public:
        FreeRTOSMutex() {
            handle_ = xSemaphoreCreateMutex();
            configASSERT(handle_ != nullptr);
        }

        ~FreeRTOSMutex() {
            if (handle_ != nullptr) {
                vSemaphoreDelete(handle_);
            }
        }

        void lock() const {
            // Block indefinitely until the mutex is acquired
            xSemaphoreTake(handle_, portMAX_DELAY);
        }

        void unlock() const {
            xSemaphoreGive(handle_);
        }

        // Non-copyable
        FreeRTOSMutex(const FreeRTOSMutex &) = delete;

        FreeRTOSMutex &operator=(const FreeRTOSMutex &) = delete;

    private:
        SemaphoreHandle_t handle_{};
    };

    class ScopedLock {
    public:
        explicit ScopedLock(const FreeRTOSMutex &m) : m_(m) { m_.lock(); }
        ~ScopedLock() { m_.unlock(); }

    private:
        const FreeRTOSMutex &m_;
    };

    /**
     * @brief Information container for a registered simple callback
     * @details Stored by value for safe copying and moving operations
     * @tparam DataT Type of the data payload in the notification
     * @tparam UserT Type of the user data passed to callback (defaults to void*)
     */
    template<typename DataT, typename UserT = void *>
    struct SimpleCallbackInfo {
        uint32_t id; ///< Unique identifier for the callback
        OnSimpleCallback<DataT, UserT> callback; ///< The callback function
        UserT userData; ///< User-defined data passed to callback
    };

    template<typename DataDt, typename UserDt = void *>
    class SimpleNotificationService {
    public:
        /**
         * @brief Constructs a new SimpleNotificationService
         * @param maxCallbacks Maximum number of callbacks that can be registered
         * 
         * @example
         * SimpleNotificationService<int> service(5); // Create service with max 5 callbacks
         */
        explicit SimpleNotificationService(const size_t maxCallbacks = 10)
            : maxCallbacks_(maxCallbacks) {
            callbacks.reserve(maxCallbacks_);
        }

        /**
         * @brief Adds a callback function to be notified when data is available
         * @param callback Function to be called when notification occurs
         * @param userData User data to be passed to callback
         * @return Unique identifier for the callback, or 0 if registration failed
         * 
         * @example
         * auto service = SimpleNotificationService<int>();
         * auto id = service.addCallback([](const int& data, void* user) {
         *     printf("Received: %d\n", data);
         * }, nullptr);
         */
        uint32_t addCallback(const OnSimpleCallback<DataDt, UserDt> &callback, const UserDt &userData) {
            ScopedLock lock(mutex_);
            if (callbacks.size() >= maxCallbacks_) {
                ESP_LOGW(TAG, "Maximum callback limit reached (%zu)", maxCallbacks_);
                return 0; // Invalid ID
            }
            static std::atomic<uint32_t> nextId{0};
            const uint32_t id = nextId.fetch_add(1);
            SimpleCallbackInfo<DataDt, UserDt> info{id, callback, userData};
            callbacks.push_back(info);
            return id;
        }

        /**
         * @brief Removes a previously registered callback
         * @param id Identifier returned by addCallback
         * @return true if callback was found and removed, false otherwise
         * 
         * @example
         * auto service = SimpleNotificationService<int>();
         * auto id = service.addCallback(...);
         * bool removed = service.eraseCallback(id);
         */
        bool eraseCallback(uint32_t id) {
            ScopedLock lock(mutex_);
            auto originalSize = callbacks.size();
            callbacks.erase(
                std::remove_if(
                    callbacks.begin(),
                    callbacks.end(),
                    [id](const auto &info) { return info.id == id; }),
                callbacks.end());
            return callbacks.size() < originalSize;
        }

        /**
         * @brief Notifies all registered callbacks with provided data
         * @param data Data to be passed to callbacks
         * 
         * @example
         * auto service = SimpleNotificationService<int>();
         * service.addCallback(...);
         * service.notify(42); // Notify all callbacks with value 42
         */
        void notify(const DataDt &data) {
            std::vector<SimpleCallbackInfo<DataDt, UserDt> > callbacksCopy; {
                ScopedLock lock(mutex_);
                if (callbacks.empty()) {
                    return;
                }
                callbacksCopy = callbacks; // Copy callbacks to execute without lock
            }

            for (const auto &info: callbacksCopy) {
                const auto &id = info.id;
                const auto &callback = info.callback;
                const auto &userdata = info.userData;
                try {
                    callback(data, userdata);
                } catch (const std::exception &e) {
                    ESP_LOGE(TAG, "Exception in callback id=%d: %s ---> Ignoring", id, e.what());
                } catch (...) {
                    ESP_LOGE(TAG, "Unknown exception in callback id=%d ---> Ignoring", id);
                }
            }
        }

        /**
         * @brief Removes all registered callbacks
         * 
         * @example
         * auto service = SimpleNotificationService<int>();
         * service.addCallback(...);
         * service.clear(); // Remove all callbacks
         */
        void clear() {
            ScopedLock lock(mutex_);
            callbacks.clear();
        }

        /**
         * @brief Gets the number of currently registered callbacks
         * @return Number of callbacks
         * 
         * @example
         * auto service = SimpleNotificationService<int>();
         * service.addCallback(...);
         * size_t count = service.getCallbackCount(); // Should return 1
         */
        size_t getCallbackCount() const {
            ScopedLock lock(mutex_);
            return callbacks.size();
        }

        /**
         * @brief Checks if a callback with given ID exists
         * @param id Identifier to check
         * @return true if callback exists, false otherwise
         * 
         * @example
         * auto service = SimpleNotificationService<int>();
         * auto id = service.addCallback(...);
         * bool exists = service.hasCallback(id); // Should return true
         */
        bool hasCallback(uint32_t id) const {
            ScopedLock lock(mutex_);
            return std::any_of(callbacks.begin(), callbacks.end(),
                               [id](const auto &info) { return info.id == id; });
        }

    private:
        size_t maxCallbacks_;
        static constexpr auto TAG{"SIMPLE_NOTIFICATION_SERVICE"};
        std::vector<SimpleCallbackInfo<DataDt, UserDt> > callbacks{};
        mutable FreeRTOSMutex mutex_;
    };
} // namespace Hilton
