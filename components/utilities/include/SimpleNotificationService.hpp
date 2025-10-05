#pragma once
#include <any>
#include <atomic>
#include <esp_log.h>
#include <functional>
#include <NotificationCallback.hpp>
#include <vector>

namespace Hilton {
    template<typename DataDt, typename UserDt = std::any>
    class SimpleNotificationService {
    public:
        SimpleNotificationService() = default;

        uint32_t addCallback(OnSimpleCallback<DataDt, std::any> &callback, const UserDt &userData = std::any()) {
            static std::atomic<uint32_t> nextId{0};
            const uint32_t id = nextId.fetch_add(1);
            SimpleCallbackInfo<DataDt, UserDt> info{id, callback, userData};
            callbacks.push_back(info);
            return id;
        }

        void eraseCallback(uint32_t id) {
            callbacks.erase(
                std::remove_if(
                    callbacks.begin(),
                    callbacks.end(),
                    [id](const auto &info) { return info.id == id; }),
                callbacks.end());
        }

        void notify(const DataDt &data) {
            if (callbacks.empty()) {
                return;
            }
            for (const auto &[id, callback, userdata]: callbacks) {
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
        std::vector<SimpleCallbackInfo<DataDt> > callbacks{};
    };
} // namespace Hilton
