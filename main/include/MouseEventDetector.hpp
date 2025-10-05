#pragma once

#include <memory>
#include <Mouse.hpp>
#include <SimpleNotificationService.hpp>

namespace Hilton {
    class MouseEventDetector {
    public:
        enum class EventType {
            ON_DATA_AVAILABLE
        };

        ~MouseEventDetector();
        MouseEventDetector();

        // Sensor-agnostic callback registered with a sensor's QueuedNotificationService
        template<typename KeyT, typename DataT, typename UserT>
        static void onSensorDataIsReady(const KeyT &key, const DataT &data, const UserT &userData);

        // our callback service for when we detect activity.
        SimpleNotificationService<std::shared_ptr<Mouse>, std::any> simpleNotificationService;

        static constexpr auto TAG{"MOUSE_EVENT_DETECTOR"};
        std::shared_ptr<Mouse> mouse;
    };
}
