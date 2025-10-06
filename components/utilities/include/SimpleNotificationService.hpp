#pragma once
#include <algorithm>
#include <any>
#include <atomic>
#include <esp_log.h>
#include <exception>
#include <functional>
#include <NotificationCallback.hpp>
#include <vector>
#include <mutex>

namespace Hilton {
    template<typename DataDt, typename UserDt = void *>
    class SimpleNotificationService {
    public:
        explicit SimpleNotificationService(const size_t maxCallbacks = 10)
            : maxCallbacks_(maxCallbacks) {
            callbacks.reserve(maxCallbacks_);
        }

        uint32_t addCallback(const OnSimpleCallback<DataDt, UserDt> &callback, const UserDt &userData) {
            std::lock_guard<std::mutex> lock(mutex_);
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

        bool eraseCallback(uint32_t id) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto originalSize = callbacks.size();
            callbacks.erase(
                std::remove_if(
                    callbacks.begin(),
                    callbacks.end(),
                    [id](const auto &info) { return info.id == id; }),
                callbacks.end());
            return callbacks.size() < originalSize;
        }

        void notify(const DataDt &data) {
            std::vector<SimpleCallbackInfo<DataDt, UserDt> > callbacksCopy; {
                std::lock_guard<std::mutex> lock(mutex_);
                if (callbacks.empty()) {
                    return;
                }
                callbacksCopy = callbacks; // Copy callbacks to execute without lock
            }

            for (const auto &[id, callback, userdata]: callbacksCopy) {
                try {
                    callback(data, userdata);
                } catch (const std::exception &e) {
                    ESP_LOGE(TAG, "Exception in callback id=%d: %s ---> Ignoring", id, e.what());
                } catch (...) {
                    ESP_LOGE(TAG, "Unknown exception in callback id=%d ---> Ignoring", id);
                }
            }
        }

        void clear() {
            std::lock_guard<std::mutex> lock(mutex_);
            callbacks.clear();
        }

        size_t getCallbackCount() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return callbacks.size();
        }

        bool hasCallback(uint32_t id) const {
            std::lock_guard<std::mutex> lock(mutex_);
            return std::any_of(callbacks.begin(), callbacks.end(),
                               [id](const auto &info) { return info.id == id; });
        }

    private:
        size_t maxCallbacks_;
        static constexpr auto TAG{"SIMPLE_NOTIFICATION_SERVICE"};
        std::vector<SimpleCallbackInfo<DataDt, UserDt> > callbacks{};
        mutable std::mutex mutex_;
    };
} // namespace Hilton
