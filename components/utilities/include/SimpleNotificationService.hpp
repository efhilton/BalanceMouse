#pragma once
#include "esp_log.h"
#include <functional>
#include <map>
#include <NotificationCallback.hpp>

namespace Hilton {
template<typename T>
class SimpleNotificationService
{
public:

    SimpleNotificationService() = default;

    int addCallback(OnDataCallback<T> &callback)
    {
        static int nextId{0};
        const int id = nextId;
        callbacks_[id] = callback;
        nextId++;
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
    std::map<size_t, OnDataCallback<T>> callbacks_{};
    size_t next_id_{0};
};
} // namespace Hilton