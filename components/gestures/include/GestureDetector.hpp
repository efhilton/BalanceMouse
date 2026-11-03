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
        static constexpr float GRAVITY_SCALE{127.0f / 9.8f}; // m/s^2 -> report units
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
         * Calibration::snapYawToRaw() is called once to break out immediately. Deliberately far
         * longer than the 20 s window test_sustained_scroll_is_not_absorbed exercises, so a real
         * sustained hold of that length is completely unaffected.
         */
        static constexpr float STUCK_SCROLL_RECOVERY_S{60.0f};

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
