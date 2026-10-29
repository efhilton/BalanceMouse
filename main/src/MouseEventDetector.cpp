#include <MouseEventDetector.hpp>
#include <esp_log.h>
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
        logCalibrationWarmup(sample);

        if (feedback != nullptr) {
            feedback->updateMotion(detector.getCalibratedTiltX(), detector.getCalibratedTiltY(),
                                   mouse.isScrollWheelActive() ? mouse.getAngle() : static_cast<int8_t>(0));
        }

        if (handled) {
            Mouse *m = &mouse;
            simpleNotificationService.notify(m);
        }
    }

    void MouseEventDetector::logCalibrationWarmup(const MotionSample &sample) {
        if (seedLogged) {
            return;
        }
        if (!calibration.isSeeded()) {
            // Boot-time only and capped, so a sensor stuck at zero can't flood the log.
            if (warmupSamplesLogged < MAX_WARMUP_SAMPLES_LOGGED) {
                ++warmupSamplesLogged;
                ESP_LOGI(TAG, "Calibration warm-up sample %d: accel=(%.2f, %.2f, %.2f) m/s^2 yaw=%.2f -> %s",
                         warmupSamplesLogged, sample.accel.x, sample.accel.y, sample.accel.z, sample.yaw,
                         Calibration::isPlausibleRestingSample(sample) ? "plausible" : "IMPLAUSIBLE");
            }
            return;
        }
        seedLogged = true;
        ESP_LOGI(TAG, "Calibration seeded after %d rejected samples: tilt baseline=(%.2f, %.2f) m/s^2, yaw baseline=%.2f",
                 calibration.samplesRejectedBeforeSeed(), calibration.tiltBaselineX(), calibration.tiltBaselineY(),
                 calibration.yawBaseline());
    }
}
