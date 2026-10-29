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
     * Seeding: the baselines are NOT taken from the very first sample. Right
     * after the IMU is reset and configured, its output registers can still hold
     * their reset value (all zero) because no conversion has completed yet, and
     * the driver's own filters (yaw integration/high-pass) are still settling.
     * Seeding from such a sample makes the board's real resting tilt look like a
     * deliberate lean that never counts as idle, so it is never adapted away
     * (phantom cursor drift until a manual re-zero). So: the first
     * `warmupSamples` samples are discarded, and the baseline is then seeded
     * from the first sample whose accelerometer magnitude is a plausible
     * resting gravity reading (see isPlausibleRestingSample()). Until seeded,
     * the calibrated values read as zero and the baselines never adapt.
     *
     * Whatever stance the board is in once seeded becomes "zero".
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

        /**
         * Samples discarded after start-up before the baseline may be seeded.
         * At the 50 Hz sampling rate this is ~200 ms on top of the driver's
         * 50 ms post-enable delay: comfortably past the QMI8658's gyro/accel
         * turn-on (tens of ms) so the data registers hold real conversions,
         * and long enough for the driver's yaw integrator (whose first sample
         * uses a ~0.5 s dt spanning the whole init sequence) to have stepped
         * past its start-up transient. Short enough to be imperceptible at boot.
         */
        static constexpr int DEFAULT_WARMUP_SAMPLES{10};

        /**
         * A resting board reads ~1 g in total. Anything outside [0.5 g, 1.5 g]
         * (in particular the all-zero reset value) is not a resting reading and
         * must not seed the baseline.
         */
        static constexpr float STANDARD_GRAVITY_MS2{9.81f};
        static constexpr float MIN_RESTING_GRAVITY_MS2{0.5f * STANDARD_GRAVITY_MS2};
        static constexpr float MAX_RESTING_GRAVITY_MS2{1.5f * STANDARD_GRAVITY_MS2};

        explicit Calibration(float tiltTimeConstantS = DEFAULT_TILT_TIME_CONSTANT_S,
                             float yawTimeConstantS = DEFAULT_YAW_TIME_CONSTANT_S,
                             int warmupSamples = DEFAULT_WARMUP_SAMPLES);

        /**
         * Record the latest raw reading. Seeds both baselines once the warm-up
         * has elapsed and the reading is a plausible resting sample.
         */
        void observe(const MotionSample &sample);

        /**
         * Adapt both baselines toward the latest raw reading by one EMA step of
         * `dtSeconds`. Does nothing when `idle` is false or before seeding.
         */
        void adapt(float dtSeconds, bool idle);

        /**
         * Instantly set both baselines to the latest raw reading. Ignored before
         * seeding (the pending seed will take a real reading anyway).
         */
        void rezero();

        [[nodiscard]] bool isSeeded() const { return seeded; }

        /** Number of observed samples that were rejected before seeding (warm-up + implausible). */
        [[nodiscard]] int samplesRejectedBeforeSeed() const { return rejectedBeforeSeed; }

        /** True if the accelerometer magnitude is finite and within the resting-gravity window. */
        [[nodiscard]] static bool isPlausibleRestingSample(const MotionSample &sample);

        [[nodiscard]] float tiltBaselineX() const { return tiltX; }
        [[nodiscard]] float tiltBaselineY() const { return tiltY; }
        [[nodiscard]] float yawBaseline() const { return yaw; }

        /** Latest raw reading minus baseline (zero until seeded). */
        [[nodiscard]] float calibratedTiltX() const { return seeded ? rawTiltX - tiltX : 0.0f; }
        [[nodiscard]] float calibratedTiltY() const { return seeded ? rawTiltY - tiltY : 0.0f; }
        [[nodiscard]] float calibratedYaw() const { return seeded ? rawYaw - yaw : 0.0f; }

    private:
        static float emaAlpha(float dtSeconds, float timeConstantS);

        void snapToRaw();

        float tiltTau;
        float yawTau;
        int warmupRemaining;
        int rejectedBeforeSeed{0};
        bool seeded{false};

        float tiltX{0};
        float tiltY{0};
        float yaw{0};

        float rawTiltX{0};
        float rawTiltY{0};
        float rawYaw{0};
    };
} // namespace Hilton
