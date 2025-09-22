#pragma once
#include "esp_log.h"
#include <NotificationCallback.hpp>
#include <any>
#include <atomic>
#include <functional>
#include <vector>

namespace Hilton {
template<typename DataDt, typename UserDt = void *>
class SimpleNotificationService
{
public:
    SimpleNotificationService() = default;

    uint32_t addCallback(OnSimpleCallback<DataDt> &callback, UserDt userData = std::any())
    {
        static std::atomic<uint32_t> nextId{0};
        const uint32_t id = nextId.fetch_add(1);
        SimpleCallbackInfo<DataDt, UserDt> info{id, callback, userData};
        callbacks_.push_back(info);
        return id;
    }

    void eraseCallback(uint32_t id)
    {
        callbacks_.erase(
            std::remove_if(
                callbacks_.begin(),
                callbacks_.end(),
                [id](const auto &info) { return info.id == id; }),
            callbacks_.end());
    }

    void notify(const DataDt &data)
    {
        if (callbacks_.empty()) {
            return;
        }
        for (const auto &[id, callback, userdata] : callbacks_) {
            try {
                callback(data, userdata);
            } catch (const std::exception &e) {
                ESP_LOGE(TAG, "Exception in callback id=%d: %s ---> Ignoring", id, e.what());
            } catch (...) {
                ESP_LOGE(TAG, "Unknown exception in callback id=%d ---> Ignoring", id);
            }
        }
    }

private:
    static constexpr auto TAG{"SIMPLE_NOTIFICATION_SERVICE"};
    std::vector<SimpleCallbackInfo<DataDt>> callbacks_{};
};
} // namespace Hilton