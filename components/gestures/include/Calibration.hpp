#pragma once

#include <MotionSample.hpp>

namespace Hilton {
    /**
     * Two independent, slowly adapting baselines:
     *   - tilt baseline: accelerometer X/Y (drives cursor movement)
     *   - yaw baseline : integrated yaw / orientation.z (drives the scroll wheel)
     *
     * Each is an exponential moving average with its own time constant. The
     * caller decides WHEN adaptation is allowed (GestureDetector only allows it
     * while fully idle: no button held, not moving, not scrolling, and both
     * calibrated signals below their *start* thresholds), so a held drag or a
     * sustained scroll is never absorbed into the baseline.
     *
     * The very first observed sample seeds both baselines instantly, so
     * whatever stance the board is in at power-on becomes "zero".
     * rezero() snaps both baselines to the latest raw reading (BOOT short press).
     */
    class Calibration {
    public:
        /**
         * Default EMA time constants. 5 s: slow enough that a brief pause
         * between gestures barely moves the baseline, fast enough that a small
         * stance offset (below the start thresholds) is ~95% absorbed after
         * ~15 s of standing still. Judgment call -- tune on hardware.
         */
        static constexpr float DEFAULT_TILT_TIME_CONSTANT_S{5.0f};
        static constexpr float DEFAULT_YAW_TIME_CONSTANT_S{5.0f};

        explicit Calibration(float tiltTimeConstantS = DEFAULT_TILT_TIME_CONSTANT_S,
                             float yawTimeConstantS = DEFAULT_YAW_TIME_CONSTANT_S);

        /** Record the latest raw reading. Seeds both baselines on the first call. */
        void observe(const MotionSample &sample);

        /**
         * Adapt both baselines toward the latest raw reading by one EMA step of
         * `dtSeconds`. Does nothing when `idle` is false or before seeding.
         */
        void adapt(float dtSeconds, bool idle);

        /** Instantly set both baselines to the latest raw reading. */
        void rezero();

        [[nodiscard]] bool isSeeded() const { return seeded; }

        [[nodiscard]] float tiltBaselineX() const { return tiltX; }
        [[nodiscard]] float tiltBaselineY() const { return tiltY; }
        [[nodiscard]] float yawBaseline() const { return yaw; }

        /** Latest raw reading minus baseline. */
        [[nodiscard]] float calibratedTiltX() const { return rawTiltX - tiltX; }
        [[nodiscard]] float calibratedTiltY() const { return rawTiltY - tiltY; }
        [[nodiscard]] float calibratedYaw() const { return rawYaw - yaw; }

    private:
        static float emaAlpha(float dtSeconds, float timeConstantS);

        float tiltTau;
        float yawTau;
        bool seeded{false};

        float tiltX{0};
        float tiltY{0};
        float yaw{0};

        float rawTiltX{0};
        float rawTiltY{0};
        float rawYaw{0};
    };
} // namespace Hilton
