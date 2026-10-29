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

        static constexpr int MAX_WARMUP_SAMPLES_LOGGED{Calibration::DEFAULT_WARMUP_SAMPLES + 5};

        Mouse &mouse;
        FeedbackState *feedback;
        Calibration calibration;
        GestureDetector detector;
        int warmupSamplesLogged{0};
        bool seedLogged{false};
    };
}
