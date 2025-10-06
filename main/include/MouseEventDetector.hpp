#pragma once

#include <memory>
#include <Mouse.hpp>
#include <Qmi8658C.hpp>
#include <SimpleNotificationService.hpp>

namespace Hilton {
    class MouseEventDetector {
    public:
        ~MouseEventDetector();

        void onSensorDataIsReady(const Qmi8658C::DataPoint &data, std::shared_ptr<Mouse> mouse);

        MouseEventDetector();

        // our callback service for when we detect activity.
        SimpleNotificationService<std::shared_ptr<Mouse> > simpleNotificationService;

        static constexpr auto TAG{"MOUSE_EVENT_DETECTOR"};
    };
}
