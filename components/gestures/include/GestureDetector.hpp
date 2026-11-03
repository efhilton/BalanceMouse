#pragma once

#include <Calibration.hpp>
#include <MotionSample.hpp>
#include <Mouse.hpp>
#include <MouseEventSink.hpp>
#include <atomic>
#include <cstdint>

namespace Hilton {
    /**
     * Turns IMU samples into discrete mouse events.
     *
     * Detection logic (thresholds, priorities, hysteresis) is the same as the
     * original MouseEventDetector; what changed is:
     *   - movement uses CALIBRATED tilt and scroll uses CALIBRATED yaw
     *     (see Calibration), instead of raw readings;
     *   - scaled values are clamped to [-127, 127] BEFORE narrowing to int8_t;
     *   - results are emitted as events to a MouseEventSink instead of being
     *     read back out of Mouse by a transport.
     *
     * Not thread-safe except requestRezero(), which may be called from any task.
     */
    class GestureDetector {
    public:
        // --- Thresholds carried over from the original MouseEventDetector ---
        static constexpr float GYRO_PRESS_THRESHOLD_DPS{100.0f};
        static constexpr uint32_t RELEASE_LOCKOUT_MS{300};
        /**
         * m/s^2 -> report units, LINEAR. No longer used for the HID movement deltas (see
         * movementDeltaFromTilt()); still used, deliberately unchanged, for the idle/adapt
         * magnitude gate in process() so calibration idle detection behaves exactly as before.
         */
        static constexpr float GRAVITY_SCALE{127.0f / 9.8f};
        /**
         * Calibration idle/adapt gate for tilt: the tilt baselines may only adapt while
         * hypot(GRAVITY_SCALE * tiltX, GRAVITY_SCALE * tiltY) is below this (linear units,
         * 13 / (127/9.8) = ~1.0 m/s^2, ~5.9 deg). This value used to be shared, by coincidence,
         * with Mouse::MOVEMENT_THRESHOLD_START (both 13). That constant is the HID cursor's
         * start hysteresis and has been retuned (#1181, Edgar: XY deadzone too large) for the
         * scaled acceleration curve; idle detection is a separate concern and must not change
         * with cursor feel, so it now has its own constant, kept at the old value (13) so
         * idle-detection behaviour is bit-for-bit unchanged.
         */
        static constexpr float TILT_IDLE_MAGNITUDE_THRESHOLD{13.0f};

        /**
         * HID movement acceleration curve (#1181, Edgar: "the further we are away from the
         * center, the more 'acceleration' the mouse should see"). FEEL-TUNING STARTING POINT,
         * NOT a derived or hardware-confirmed value -- both the full-scale angle and the
         * exponent are expected to be adjusted (together) after Edgar tries it on a real host.
         * Affects ONLY the HID movement deltas: the LED ball (a position indicator), the scroll
         * wheel and the idle/adapt gate are all unaffected. See movementDeltaFromTilt().
         *
         * Tilt angle (degrees) at which the cursor reaches full speed (report
         * round(127 * MOVEMENT_SPEED_SCALE) = 64 with the current 0.5 scale). Reuses the
         * device's established "full tilt" angle, FeedbackRenderer::BALL_FULL_SCALE_TILT_DEG
         * (the LED ball reaches the matrix edge at the same tilt), so ball-at-edge and
         * cursor-at-full-speed coincide. Duplicated rather than shared because led_feedback
         * does not (and should not need to) depend on the gestures component;
         * test_movement_full_scale_matches_ball_full_scale (host_test) fails if the two drift
         * apart -- change them together, or deliberately decouple them and update that test.
         */
        static constexpr float MOVEMENT_FULL_SCALE_TILT_DEG{20.0f};
        /**
         * m/s^2 equivalent of MOVEMENT_FULL_SCALE_TILT_DEG: STANDARD_GRAVITY_MS2 * sin(20 deg)
         * = 9.81 * 0.34202 = 3.3552 (same derivation as FeedbackRenderer::BALL_FULL_SCALE_TILT).
         * Recompute if MOVEMENT_FULL_SCALE_TILT_DEG changes (guarded by host_test).
         */
        static constexpr float MOVEMENT_FULL_SCALE_TILT{3.3552f};
        /**
         * Response-curve exponent applied to the normalised tilt magnitude. 1.0 = linear (no
         * acceleration); 2.0 = quadratic (default: half of full tilt -> 1/4 of full speed,
         * fine control near centre); higher = even gentler centre and steeper edges. Judgment
         * call, tune by feel.
         */
        static constexpr float MOVEMENT_ACCEL_EXPONENT{2.0f};
        /**
         * Uniform output scale applied to the HID movement curve, AFTER the curve shape (exponent
         * and full-scale angle, both unchanged) and BEFORE rounding. Edgar's real-hardware feedback
         * on the first curve (full speed = 127): "for X and Y, please half the acceleration. It's
         * crazy fast." -> 0.5, i.e. the same quadratic ramp, half the speed at every tilt; full
         * tilt (20 deg) now gives +/-64 instead of +/-127. Tunable further if he wants more or less
         * than exactly half. Note the Mouse start/stop hysteresis (Mouse::MOVEMENT_THRESHOLD_START/
         * STOP) is applied to the scaled output, so changing this scale changes the deadzone angle
         * unless those thresholds are re-derived too (they were: 13/5 -> 6/2, see Mouse.hpp;
         * guarded by test_movement_start_stop_thresholds_match_intended_deadzone).
         */
        static constexpr float MOVEMENT_SPEED_SCALE{0.5f};

        /**
         * Maps ONE axis of calibrated tilt (m/s^2, signed) to an HID movement delta:
         *   n     = clamp(|tilt| / MOVEMENT_FULL_SCALE_TILT, 0, 1)
         *   delta = sign(tilt) * round(n ^ MOVEMENT_ACCEL_EXPONENT * 127 * MOVEMENT_SPEED_SCALE)
         * With the current 0.5 scale the output range is -64..+64 (reached at 20 deg and beyond).
         * The curve is applied to the MAGNITUDE only and the sign multiplied back in afterwards
         * (raising a signed value to a power would lose the sign for even exponents). Clamped
         * via Mouse::saturateToReport. NaN -> 0. Pure, so directly testable.
         */
        [[nodiscard]] static int8_t movementDeltaFromTilt(float tilt);
        /** Z-rotation angle that saturates the scroll report range (-127..127). Tunable (#1181,
         * Edgar). Changing this moves Mouse::ANGLE_THRESHOLD_START/STOP's effective degree
         * meaning too -- see the derivation comment there and
         * test_scroll_engage_disengage_thresholds_match_mouse_hysteresis, which fails loudly if
         * the two drift apart. */
        static constexpr float MAX_SCROLL_ANGLE_DEG{80.0f};
        static constexpr float SCROLL_SCALE{127.0f / MAX_SCROLL_ANGLE_DEG}; // deg -> report units

        /**
         * Scroll engagement Schmitt trigger, stated in real degrees (#1181, Edgar: "Schmitt
         * trigger... engage going up stays off until abs(angle) hits 10, but once engaged,
         * don't disengage again until abs(angle) drops back down to about 5" -- a real deadzone,
         * not a single threshold, so a reading sitting right at the edge can't chatter on/off).
         * The actual enforcement is Mouse::ANGLE_THRESHOLD_START/STOP, in report units (Mouse
         * has no notion of degrees); these two floats exist so the degree intent is named and
         * testable in one place instead of only living as a comment next to a magic int8.
         */
        static constexpr float SCROLL_ENGAGE_DEG{10.0f};
        static constexpr float SCROLL_DISENGAGE_DEG{5.0f};

        /**
         * Stuck-scroll recovery timeout (#1181, Edgar's real-hardware drift find): gyro Z has
         * no accelerometer-style absolute reference, so its integration drifts unboundedly even
         * at rest (measured ~0.5-0.6 deg/s on the real board). If that drift alone ever pushes
         * calibratedYaw past SCROLL_ENGAGE_DEG, the scroll wheel latches active and, by design
         * (see Calibration::adapt()'s doc comment -- a real sustained scroll must never be
         * absorbed into the baseline, no matter how long it's held), yaw can never adapt its
         * way back out: a one-way ratchet with no gradual fix compatible with that guarantee.
         *
         * If "wheel active, no button held, cursor not moving, gyro quiet" persists
         * continuously for longer than this many seconds, it is treated as certainly NOT a real
         * gesture -- no plausible deliberate hold lasts this long with zero gyro movement -- and
         * Calibration::snapYawToRaw() is called once to break out immediately.
         *
         * 30 s (was 60 s): Edgar's explicit product decision (#1181, 2026-11-03), chosen over
         * both 60 s (safe but slow to recover from a large misalignment) and 10 s (fast, but
         * would cut off any real scroll held longer than 10 s). It is still above the 20 s
         * window test_sustained_scroll_is_not_absorbed exercises, so a real sustained hold of
         * that length is unaffected -- but the margin is now only 10 s (it was 40 s at 60 s).
         * A real quiet hold longer than 30 s WILL be snapped; keep the test window comfortably
         * below this value if either is ever changed.
         */
        static constexpr float STUCK_SCROLL_RECOVERY_S{30.0f};

        /**
         * Wheel ticks per second per scroll-angle unit (angle is -127..127).
         * At the start threshold (~16 report units, just past SCROLL_ENGAGE_DEG) that's ~5
         * ticks/s, at full deflection ~38 ticks/s. Judgment call: the original code had no
         * wheel-rate mapping.
         */
        static constexpr float SCROLL_TICKS_PER_SECOND_PER_UNIT{0.3f};

        /** Used for the first sample and to cap dt after a stall. */
        static constexpr float NOMINAL_SAMPLE_PERIOD_S{0.02f};
        static constexpr float MAX_SAMPLE_PERIOD_S{0.1f};

        /**
         * `clicksEnabled` (default true) gates handleButtonClick only -- scroll and movement
         * detection are unaffected either way. Exists so a caller can temporarily silence
         * gyro-triggered clicks without touching detection logic (#1181, Edgar: clicks made
         * it harder to isolate a movement-axis bug under live debugging, since the same tilt
         * gestures used to test movement also cross the click gyro threshold).
         */
        GestureDetector(Mouse &mouse, Calibration &calibration, MouseEventSink *sink, bool clicksEnabled = true);

        /**
         * Process one sample. Returns true when one of the handlers (scroll,
         * button, movement) reported activity -- the same meaning as the old
         * ChainOfResponsibility result. Always false (and nothing is emitted)
         * until the calibration is seeded after its start-up warm-up.
         */
        bool process(const MotionSample &sample);

        /** Ask for an immediate re-zero of both baselines on the next sample. Thread-safe. */
        void requestRezero() { rezeroRequested.store(true); }

        /** True if the last processed sample counted as idle (baselines allowed to adapt). */
        [[nodiscard]] bool wasIdle() const { return lastIdle; }

        [[nodiscard]] float getCalibratedTiltX() const { return calibration.calibratedTiltX(); }
        [[nodiscard]] float getCalibratedTiltY() const { return calibration.calibratedTiltY(); }
        [[nodiscard]] float getCalibratedYaw() const { return calibration.calibratedYaw(); }

    private:
        bool handleScrollWheel(float calibratedYaw, float dtSeconds);

        bool handleButtonClick(const MotionSample &s);

        bool handleMouseMovement(float calibratedX, float calibratedY);

        void emitButtonTransitions(const bool (&before)[3]);

        [[nodiscard]] bool anyButtonPressed() const;

        Mouse &mouse;
        Calibration &calibration;
        MouseEventSink *sink;

        bool clicksEnabled;
        std::atomic<bool> rezeroRequested{false};
        bool hasPreviousSample{false};
        uint32_t previousTimestampMs{0};
        bool hasReleased{false};
        uint32_t lastReleaseMs{0};
        float scrollAccumulator{0};
        bool lastIdle{false};
        /** Seconds the wheel has been continuously active+quiet; see STUCK_SCROLL_RECOVERY_S. */
        float stuckScrollSeconds{0};
    };
} // namespace Hilton
