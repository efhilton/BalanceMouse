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

    bool GestureDetector::handleScrollWheel(const float calibratedYaw, const float dtSeconds) {
        if (anyButtonPressed() || mouse.isMoving()) {
            scrollAccumulator = 0;
            return false;
        }

        // Use calibrated yaw (orientation Z) for the scroll wheel. Clamp before the narrowing cast.
        mouse.setAngle(Mouse::saturateToReport(SCROLL_SCALE * calibratedYaw));

        if (!mouse.isScrollWheelActive()) {
            scrollAccumulator = 0;
            return false;
        }

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

    bool GestureDetector::handleMouseMovement(const float calibratedX, const float calibratedY) {
        if (mouse.isScrollWheelActive()) {
            return false;
        }

        // Clamp to the report range BEFORE narrowing to int8_t (see Mouse::saturateToReport).
        mouse.updateMovement(Mouse::saturateToReport(GRAVITY_SCALE * calibratedX),
                             Mouse::saturateToReport(GRAVITY_SCALE * calibratedY));

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
        const float tiltUnits = std::hypot(GRAVITY_SCALE * tiltX, GRAVITY_SCALE * tiltY);
        const float yawUnits = std::abs(SCROLL_SCALE * yaw);
        const bool tiltIdle = !anyButtonPressed()
                              && !mouse.isScrollWheelActive()
                              && tiltUnits < static_cast<float>(Mouse::MOVEMENT_THRESHOLD_START);
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
