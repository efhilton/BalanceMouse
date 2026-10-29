#include <MouseEventDetector.hpp>
#include <esp_timer.h>

namespace Hilton {
    MouseEventDetector::MouseEventDetector(Mouse &mouse, MouseEventSink *sink, FeedbackState *feedback)
        : mouse(mouse), feedback(feedback), calibration(), detector(mouse, calibration, sink) {
    }

    MouseEventDetector::~MouseEventDetector() = default;

    void MouseEventDetector::onSensorDataIsReady(const Qmi8658C::DataPoint &data) {
        MotionSample sample{};
        sample.accel = {data.rawAccel.x, data.rawAccel.y, data.rawAccel.z};
        sample.gyro = {data.rawGyro.x, data.rawGyro.y, data.rawGyro.z};
        sample.yaw = data.orientation.z;
        sample.timestampMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        // Dispatch order inside GestureDetector::process: scroll wheel, then buttons, then movement.
        const bool handled = detector.process(sample);

        if (feedback != nullptr) {
            feedback->updateMotion(detector.getCalibratedTiltX(), detector.getCalibratedTiltY(),
                                   mouse.isScrollWheelActive() ? mouse.getAngle() : static_cast<int8_t>(0));
        }

        if (handled) {
            Mouse *m = &mouse;
            simpleNotificationService.notify(m);
        }
    }
}
