#pragma once
#include "esp_log.h"
#include <functional>
#include <map>

namespace Hilton {
template<typename T>
class SimpleNotificationService
{
public:
    using OnDataCallback = std::function<void(const T &data)>;

    SimpleNotificationService() = default;

    size_t addCallback(OnDataCallback &callback)
    {
        size_t id = next_id_++;
        callbacks_[id] = callback;
        return id;
    }

    void eraseCallback(size_t id) { callbacks_.erase(id); }

    void notify(const T &data)
    {
        if (callbacks_.empty()) {
            return;
        }
        for (auto &[id, callback] : callbacks_) {
            try {
                callback(data);
            } catch (const std::exception &e) {
                ESP_LOGE(TAG, "Exception in callback id=%d: %s ---> Ignoring", id, e.what());
            } catch (...) {
                ESP_LOGE(TAG, "Unknown exception in callback id=%d ---> Ignoring", id);
            }
        }
    }

private:
    static constexpr auto TAG{"SIMPLE_NOTIFICATION_SERVICE"};
    std::map<size_t, OnDataCallback> callbacks_{};
    size_t next_id_{0};
};
} // namespace Hilton