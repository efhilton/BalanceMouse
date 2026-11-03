#pragma once

#include <Calibration.hpp>
#include <FeedbackState.hpp>
#include <GestureDetector.hpp>
#include <Mouse.hpp>
#include <MouseEventSink.hpp>
#include <Qmi8658C.hpp>
#include <SimpleNotificationService.hpp>

namespace Hilton {
    /**
     * Device-side adapter: converts Qmi8658C::DataPoint into a MotionSample, runs the
     * portable GestureDetector (which emits events to `sink`), and pushes calibrated
     * tilt / scroll state to the LED feedback tracker.
     *
     * All gesture logic lives in components/gestures so it can be host-tested.
     */
    class MouseEventDetector {
    public:
        MouseEventDetector(Mouse &mouse, MouseEventSink *sink, FeedbackState *feedback);

        ~MouseEventDetector();

        /** Called from the Qmi8658C sampling task. */
        void onSensorDataIsReady(const Qmi8658C::DataPoint &data);

        /** BOOT short press: re-zero both calibration baselines on the next sample. Thread-safe. */
        void requestRezero() { detector.requestRezero(); }

        // Notified (from the sampling task) whenever a handler reported activity.
        SimpleNotificationService<Mouse *> simpleNotificationService;

        static constexpr auto TAG{"MOUSE_EVENT_DETECTOR"};

    private:
        /** Boot-time log of the calibration warm-up samples and the seeded baseline. */
        void logCalibrationWarmup(const MotionSample &sample);

        /**
         * Periodic post-seed log of calibrated tilt X/Y and yaw (#1181, Edgar/Maria,
         * 2026-11-02): lets the axis-swap fix, and any other orientation behaviour, be
         * verified by reading numeric values over serial -- without needing HID movement
         * re-enabled (that transport stays fully off per Main.cpp). Axis convention is Edgar's
         * authoritative one (right-hand rule, X=right, Y=forward, Z=up): rotation about X
         * should move only the calibrated Y value, rotation about Y should move only the
         * calibrated X value, rotation about Z should only affect scroll/yaw. Separate from
         * logCalibrationWarmup(), which only covers the one-time seed.
         */
        void logCalibratedTiltForVerification(const MotionSample &sample);

        static constexpr int MAX_WARMUP_SAMPLES_LOGGED{Calibration::DEFAULT_WARMUP_SAMPLES + 5};
        static constexpr uint32_t TILT_LOG_PERIOD_MS{200};

        Mouse &mouse;
        FeedbackState *feedback;
        Calibration calibration;
        GestureDetector detector;
        int warmupSamplesLogged{0};
        bool seedLogged{false};
        uint32_t lastTiltLogMs{0};
    };
}
