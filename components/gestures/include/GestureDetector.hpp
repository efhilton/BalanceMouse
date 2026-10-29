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
        static constexpr float MAX_SCROLL_ANGLE_DEG{60.0f};
        static constexpr float SCROLL_SCALE{127.0f / MAX_SCROLL_ANGLE_DEG}; // deg -> report units

        /**
         * Wheel ticks per second per scroll-angle unit (angle is -127..127).
         * At the start threshold (16) that's ~5 ticks/s, at full deflection ~38
         * ticks/s. Judgment call: the original code had no wheel-rate mapping.
         */
        static constexpr float SCROLL_TICKS_PER_SECOND_PER_UNIT{0.3f};

        /** Used for the first sample and to cap dt after a stall. */
        static constexpr float NOMINAL_SAMPLE_PERIOD_S{0.02f};
        static constexpr float MAX_SAMPLE_PERIOD_S{0.1f};

        GestureDetector(Mouse &mouse, Calibration &calibration, MouseEventSink *sink);

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

        std::atomic<bool> rezeroRequested{false};
        bool hasPreviousSample{false};
        uint32_t previousTimestampMs{0};
        bool hasReleased{false};
        uint32_t lastReleaseMs{0};
        float scrollAccumulator{0};
        bool lastIdle{false};
    };
} // namespace Hilton
