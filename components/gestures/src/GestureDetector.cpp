#include <GestureDetector.hpp>
#include <cmath>

namespace Hilton {
    using Button = MouseButtonStateMachine::Button;
    using MouseButtonStateMachine::Event::MouseDown;
    using MouseButtonStateMachine::Event::MouseUp;

    static constexpr Button ALL_BUTTONS[3]{Button::Left, Button::Right, Button::Middle};

    GestureDetector::GestureDetector(Mouse &mouse, Calibration &calibration, MouseEventSink *sink,
                                      const bool clicksEnabled)
        : mouse(mouse), calibration(calibration), sink(sink), clicksEnabled(clicksEnabled) {
    }

    bool GestureDetector::anyButtonPressed() const {
        return mouse.isButtonPressed(Button::Left)
               || mouse.isButtonPressed(Button::Right)
               || mouse.isButtonPressed(Button::Middle);
    }

    /*
     * DELIBERATE, DATA-DRIVEN DECISION -- do NOT re-add `|| mouse.isMoving()` to the bail-out
     * below (#1181, Edgar, 2026-11-02: "scroll won't engage, the cursor moves instead").
     *
     * This function used to bail out while the cursor was moving. Live ON-state captures from the
     * real board showed that twisting it around Z (the scroll gesture) also tips it sideways by
     * 10-15 deg. While scroll was actively engaged, side tilt on the linear GRAVITY_SCALE metric
     * had a median of 24.6 and a 90th percentile of 38.7. In 87% of those samples it was above the
     * old pre-shrink movement-start magnitude (13). That side tilt reached the movement
     * hysteresis first, latched mouse.isMoving(), and the old check then blocked scroll. So scroll
     * lost to movement even though it is dispatched FIRST in process() (scroll, clicks, movement).
     * No tilt-magnitude threshold can fix this: the incidental tilt that comes with a Z-twist is
     * usually LARGER than a deliberate cursor tilt, so the two cannot be told apart by size.
     *
     * The reverse risk is low. Tilting in X/Y to move the cursor does not spin the board around
     * Z, so a deliberate cursor move being taken over by an incidental yaw blip is far less likely
     * than the failure that was reported. Removing the check makes the existing priority order
     * hold in practice. Buttons still block scroll (a held button means a drag).
     *
     * Unchanged: while scroll is active, movement output stays fully suppressed (process() skips
     * handleMouseMovement when handled is true, and handleMouseMovement also bails out itself).
     */
    bool GestureDetector::handleScrollWheel(const float calibratedYaw, const float dtSeconds) {
        if (anyButtonPressed()) {
            scrollAccumulator = 0;
            return false;
        }

        // Use calibrated yaw (orientation Z) for the scroll wheel. Clamp before the narrowing cast.
        // NOTE: the sign is deliberately NOT flipped here (nor on the emitted ticks below): both
        // mouse.getAngle() (-> FeedbackState::updateMotion) and the ticks (-> FeedbackState::onScroll
        // via the event fan-out) feed the hardware-confirmed scroll chevron, which assumes they
        // share calibratedYaw's sign. The HID wheel sign correction lives in the HID transport
        // only -- see MouseHidReportBuilder::onScroll (MouseHidReport.hpp).
        mouse.setAngle(Mouse::saturateToReport(SCROLL_SCALE * calibratedYaw));

        if (!mouse.isScrollWheelActive()) {
            scrollAccumulator = 0;
            return false;
        }

        // Scroll can now engage while the movement hysteresis is latched on (see the doc comment
        // above). handleMouseMovement is skipped for as long as scroll is active, so nothing else
        // would ever clear that latch. A stale isMoving() would block the stuck-scroll recovery
        // (stuckAndQuiet in process()) and the yaw idle gate. Feeding a zero delta releases it
        // (0 < MOVEMENT_THRESHOLD_STOP) and zeroes the stored deltas. After scroll disengages,
        // movement restarts from rest and must cross MOVEMENT_THRESHOLD_START again.
        mouse.updateMovement(0, 0);

        // Convert the deflection into a wheel-tick rate and emit whole ticks.
        scrollAccumulator += static_cast<float>(mouse.getAngle()) * SCROLL_TICKS_PER_SECOND_PER_UNIT * dtSeconds;
        const float whole = std::trunc(scrollAccumulator);
        if (whole != 0.0f) {
            scrollAccumulator -= whole;
            const int8_t ticks = Mouse::saturateToReport(whole);
            if (sink != nullptr) {
                sink->onScroll(ticks);
            }
        }
        return true;
    }

    bool GestureDetector::handleButtonClick(const MotionSample &s) {
        if (mouse.isScrollWheelActive()) {
            return false;
        }

        const float gyroX = s.gyro.x; // Roll angular speed
        const float gyroY = s.gyro.y; // Pitch angular speed
        const bool isLeftRightClick = std::abs(gyroX) > GYRO_PRESS_THRESHOLD_DPS;
        const bool isVerticalClick = std::abs(gyroY) > GYRO_PRESS_THRESHOLD_DPS;
        const bool allButtonsIdle = mouse.isButtonIdle(Button::Left)
                                    && mouse.isButtonIdle(Button::Right)
                                    && mouse.isButtonIdle(Button::Middle);
        const bool lockoutOver = !hasReleased || (s.timestampMs - lastReleaseMs) > RELEASE_LOCKOUT_MS;

        // Listen for press events
        if (allButtonsIdle && lockoutOver) {
            if (isLeftRightClick) {
                if (gyroX < 0) {
                    mouse.getButton(Button::Left).handleEvent(MouseDown);
                    return true;
                }
                if (gyroX > 0) {
                    mouse.getButton(Button::Right).handleEvent(MouseDown);
                    return true;
                }
            }
            if (isVerticalClick && gyroY < 0) {
                mouse.getButton(Button::Middle).handleEvent(MouseDown);
                return true;
            }
        }

        // Listen for release events (only a fast roll/pitch back releases)
        if (isLeftRightClick) {
            if (gyroX > 0 && mouse.isButtonPressed(Button::Left)) {
                mouse.getButton(Button::Left).handleEvent(MouseUp);
                hasReleased = true;
                lastReleaseMs = s.timestampMs;
                return true;
            }
            if (gyroX < 0 && mouse.isButtonPressed(Button::Right)) {
                mouse.getButton(Button::Right).handleEvent(MouseUp);
                hasReleased = true;
                lastReleaseMs = s.timestampMs;
                return true;
            }
        }
        if (isVerticalClick && gyroY > 0 && mouse.isButtonPressed(Button::Middle)) {
            mouse.getButton(Button::Middle).handleEvent(MouseUp);
            hasReleased = true;
            lastReleaseMs = s.timestampMs;
            return true;
        }

        return false;
    }

    int8_t GestureDetector::movementDeltaFromTilt(const float tilt) {
        float n = std::abs(tilt) / MOVEMENT_FULL_SCALE_TILT;
        if (!(n > 0.0f)) {
            return 0; // zero or NaN
        }
        if (n > 1.0f) {
            n = 1.0f;
        }
        // Magnitude only (always >= 0), so the power can never flip or drop the sign. The speed
        // scale is a positive scalar applied BEFORE rounding, so it can't flip the sign either and
        // low magnitudes still round to the nearest integer rather than being truncated.
        const float magnitude = std::round(std::pow(n, MOVEMENT_ACCEL_EXPONENT)
                                           * Mouse::REPORT_MAX * MOVEMENT_SPEED_SCALE);
        return Mouse::saturateToReport(tilt < 0.0f ? -magnitude : magnitude);
    }

    bool GestureDetector::handleMouseMovement(const float calibratedX, const float calibratedY) {
        if (mouse.isScrollWheelActive()) {
            return false;
        }

        // HID cursor sign mapping (#1181, Edgar's explicit final instruction after real-hardware
        // testing, 2026-11-02):
        //   X: calibratedX -> dx unchanged.
        //   Y: calibratedY -> dy NEGATED.
        // Scoped to the HID movement deltas only. The LED display's Y handling
        // (FeedbackState::updateMotion, state.tiltY = -tiltY) is a separate path, independently
        // confirmed correct, and must not be changed to match or compensate for this. Mouse's
        // hysteresis and the idle check use magnitude only; movementDeltaFromTilt curves the
        // magnitude and preserves the sign it is given.
        mouse.updateMovement(movementDeltaFromTilt(calibratedX),
                             movementDeltaFromTilt(-calibratedY));

        if (mouse.isMoving() && sink != nullptr) {
            const auto &m = mouse.getMovement();
            if (m.deltaX != 0 || m.deltaY != 0) {
                sink->onMovement(m.deltaX, m.deltaY);
            }
        }
        return mouse.isMoving();
    }

    void GestureDetector::emitButtonTransitions(const bool (&before)[3]) {
        if (sink == nullptr) {
            return;
        }
        for (int i = 0; i < 3; ++i) {
            const bool now = mouse.isButtonPressed(ALL_BUTTONS[i]);
            if (now && !before[i]) {
                sink->onButtonDown(ALL_BUTTONS[i]);
            } else if (!now && before[i]) {
                sink->onButtonUp(ALL_BUTTONS[i]);
            }
        }
    }

    bool GestureDetector::process(const MotionSample &sample) {
        float dt = NOMINAL_SAMPLE_PERIOD_S;
        if (hasPreviousSample) {
            dt = static_cast<float>(sample.timestampMs - previousTimestampMs) / 1000.0f;
            if (dt < 0.0f) dt = 0.0f;
            if (dt > MAX_SAMPLE_PERIOD_S) dt = MAX_SAMPLE_PERIOD_S;
        }
        hasPreviousSample = true;
        previousTimestampMs = sample.timestampMs;

        calibration.observe(sample);
        if (rezeroRequested.exchange(false)) {
            calibration.rezero();
        }

        // Until the baselines are seeded from a real resting reading (start-up warm-up,
        // see Calibration), emit nothing: no movement, scroll or clicks from transients.
        if (!calibration.isSeeded()) {
            lastIdle = false;
            return false;
        }

        const float tiltX = calibration.calibratedTiltX();
        const float tiltY = calibration.calibratedTiltY();
        const float yaw = calibration.calibratedYaw();

        bool before[3];
        for (int i = 0; i < 3; ++i) {
            before[i] = mouse.isButtonPressed(ALL_BUTTONS[i]);
        }

        // Dispatch order: scroll wheel first, then buttons, then movement. The
        // first handler that reports activity ends the chain for this sample,
        // so an active scroll suppresses button and movement detection.
        // (Scroll itself is suppressed while a button is held or the cursor is moving.)
        bool handled = handleScrollWheel(yaw, dt);
        if (!handled && clicksEnabled) {
            handled = handleButtonClick(sample);
        }
        if (!handled) {
            handled = handleMouseMovement(tiltX, tiltY);
        }

        emitButtonTransitions(before);

        // Baselines may only adapt while completely idle: no button held (so a drag is never
        // absorbed), not moving, not scrolling, and both calibrated signals below their START
        // thresholds. Unchanged from the original code (now just two independently-computed
        // flags instead of one shared bool -- see Calibration::adapt()'s doc comment).
        // Deliberately still the LINEAR GRAVITY_SCALE magnitude, not the HID acceleration curve
        // (movementDeltaFromTilt): idle detection semantics are independent of cursor feel. For the
        // same reason the tilt gate uses its own TILT_IDLE_MAGNITUDE_THRESHOLD, not the HID cursor
        // hysteresis Mouse::MOVEMENT_THRESHOLD_START (decoupled in #1181; see the constant's doc).
        const float tiltUnits = std::hypot(GRAVITY_SCALE * tiltX, GRAVITY_SCALE * tiltY);
        const float yawUnits = std::abs(SCROLL_SCALE * yaw);
        const bool tiltIdle = !anyButtonPressed()
                              && !mouse.isScrollWheelActive()
                              && tiltUnits < TILT_IDLE_MAGNITUDE_THRESHOLD;
        const bool yawIdle = !anyButtonPressed()
                             && !mouse.isMoving()
                             && !mouse.isScrollWheelActive()
                             && yawUnits < static_cast<float>(Mouse::ANGLE_THRESHOLD_START);
        lastIdle = tiltIdle && yawIdle;
        calibration.adapt(dt, tiltIdle, yawIdle);

        // Stuck-scroll recovery timeout (#1181, Edgar's real-hardware drift find -- see
        // STUCK_SCROLL_RECOVERY_S's doc comment for the full story). Tracked as a running
        // duration rather than folded into yawIdle/adapt() because no gradual EMA correction
        // can satisfy both "zero drift during any real sustained scroll" and "eventually
        // recovers" at once -- this is a one-time, decisive snap instead, gated on having seen
        // this exact state continuously for far longer than any real gesture plausibly lasts.
        const bool stuckAndQuiet = mouse.isScrollWheelActive()
                                   && !anyButtonPressed()
                                   && !mouse.isMoving()
                                   && std::abs(sample.gyro.z) < Calibration::MAX_SEED_GYRO_DPS;
        if (stuckAndQuiet) {
            stuckScrollSeconds += dt;
            if (stuckScrollSeconds > STUCK_SCROLL_RECOVERY_S) {
                calibration.snapYawToRaw();
                stuckScrollSeconds = 0.0f;
            }
        } else {
            stuckScrollSeconds = 0.0f;
        }

        return handled;
    }
} // namespace Hilton
