#include <MouseEventDetector.hpp>
#include <esp_log.h>
#include <esp_timer.h>

namespace Hilton {
    MouseEventDetector::MouseEventDetector(Mouse &mouse, MouseEventSink *sink, FeedbackState *feedback)
        // Clicks run fully enabled (#1181): every gesture, including clicks, goes out over HID
        // and shows up on the LED display. GestureDetector's clicksEnabled ctor param (default
        // true, unused below) is left in place in case clicks alone ever need muting
        // independent of the HID transport.
        : mouse(mouse), feedback(feedback), calibration(), detector(mouse, calibration, sink) {
    }

    MouseEventDetector::~MouseEventDetector() = default;

    void MouseEventDetector::onSensorDataIsReady(const Qmi8658C::DataPoint &data) {
        MotionSample sample{};
        // Axis remap, not a straight copy -- see MotionSample::accel's doc comment for the
        // authoritative convention (#1181, Edgar, right-hand rule: X=right, Y=forward, Z=up).
        // The QMI8658 is mounted with its own raw x/y swapped relative to the board's logical
        // X/Y: raw x responds to rotation about logical Y (so it must feed accel.y, the
        // up/down channel) and raw y responds to rotation about logical X (so it must feed
        // accel.x, the left/right channel) -- a straight-through copy silently swapped them.
        // Confirmed against Edgar's real-hardware report before the fix: pitching forward
        // (rotation about X) moved the ball/cursor sideways instead of up/down, and rolling
        // left/right (rotation about Y) produced a small forward/back nudge instead of
        // left/right -- exactly this swap. Re-confirmed correct post-fix via Edgar's hardware
        // pass: click-and-drag (which exercises this same movement path) now works as intended.
        sample.accel = {data.rawAccel.y, data.rawAccel.x, data.rawAccel.z};
        sample.gyro = {data.rawGyro.x, data.rawGyro.y, data.rawGyro.z};
        sample.yaw = data.orientation.z;
        sample.timestampMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        // Dispatch order inside GestureDetector::process: scroll wheel, then buttons, then movement.
        const bool handled = detector.process(sample);
        logCalibrationWarmup(sample);
        logCalibratedTiltForVerification(sample);

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
            // Settling now takes up to Calibration::SETTLING_SAMPLES plausible readings, not one
            // (#1181, Edgar) -- a periodic progress line so "why hasn't it centred yet" during
            // that ~10 s window is visible without another flash cycle to find out.
            const int settled = calibration.settledSampleCount();
            if (settled > 0 && settled % 100 == 0) {
                ESP_LOGI(TAG, "Calibration settling: %d/%d plausible samples averaged so far",
                         settled, Calibration::SETTLING_SAMPLES);
            }
            return;
        }
        seedLogged = true;
        ESP_LOGI(TAG, "Calibration seeded after %d rejected samples: tilt baseline=(%.2f, %.2f) m/s^2, yaw baseline=%.2f"
                 " deg, gyroZ bias=%.3f dps",
                 calibration.samplesRejectedBeforeSeed(), calibration.tiltBaselineX(), calibration.tiltBaselineY(),
                 calibration.yawBaseline(), calibration.yawRateBiasDps());
    }

    void MouseEventDetector::logCalibratedTiltForVerification(const MotionSample &sample) {
        if (!calibration.isSeeded()) {
            return;
        }
        if (sample.timestampMs - lastTiltLogMs < TILT_LOG_PERIOD_MS) {
            return;
        }
        lastTiltLogMs = sample.timestampMs;
        // Labels use Edgar's authoritative axis convention (#1181, right-hand rule: X=right,
        // Y=forward, Z=up), not "roll/pitch": X moves from rotation about Y, Y moves from
        // rotation about X, yaw/scroll moves from rotation about Z.
        // Trailing gate fields (#1181, Edgar: scroll never engages on hardware): everything
        // GestureDetector::handleScrollWheel() checks before it can engage -- cursor "moving"
        // latch, any button held, wheel state/angle -- plus live gyro Z (proves the Z rate
        // sensor responds at all). Appended after the original fields so existing greps on the
        // VERIFY prefix still match.
        ESP_LOGI(TAG, "VERIFY calibratedX[rotY]=%.2f calibratedY[rotX]=%.2f yaw[rotZ,scroll]=%.2f m/s^2 deg"
                 " | gyroZ=%.1f dps moving=%d btn(L,R,M)=%d%d%d scroll=%s angle=%d",
                 detector.getCalibratedTiltX(), detector.getCalibratedTiltY(), detector.getCalibratedYaw(),
                 sample.gyro.z, mouse.isMoving() ? 1 : 0,
                 mouse.isButtonPressed(MouseButtonStateMachine::Button::Left) ? 1 : 0,
                 mouse.isButtonPressed(MouseButtonStateMachine::Button::Right) ? 1 : 0,
                 mouse.isButtonPressed(MouseButtonStateMachine::Button::Middle) ? 1 : 0,
                 mouse.isScrollWheelActive() ? "ON" : "off", static_cast<int>(mouse.getAngle()));
    }
}
