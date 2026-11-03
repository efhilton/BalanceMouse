#pragma once

#include <MotionSample.hpp>
#include <cstdint>

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
     * `warmupSamples` samples are discarded, and the baseline is then seeded from
     * the AVERAGE of the next SETTLING_SAMPLES samples that are each a plausible
     * resting reading (see isPlausibleRestingSample()) -- not from a single such
     * sample. Until seeded, the calibrated values read as zero and the baselines
     * never adapt.
     *
     * Averaging instead of single-sample seeding (#1181, Edgar's real-hardware
     * finding): a single lucky instant that is simultaneously gravity-plausible
     * AND gyro-quiet is hard to guarantee on request if the board cannot be laid
     * perfectly flat, or sits somewhere with a little ambient vibration. A sample
     * that fails the plausibility check (still being rotated, implausible gravity)
     * is skipped without resetting progress, so brief jostling just makes settling
     * take a little longer rather than failing outright or baking in a bad instant.
     * This turns "wait for one lucky sample" into "add up SETTLING_SAMPLES good
     * samples, however long that takes" -- the baseline settles onto wherever the
     * board is actually sitting without needing a single discrete clean moment.
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
        /**
         * Yaw EMA time constant: 3 s (was 5 s). 2026-11-02 (#1181), Edgar after a real-hardware
         * test: "The Z axis needs to adapt faster... within 10 seconds, that direction becomes our
         * new zero." A first-order EMA covers 1 - e^(-t/tau) of a step, i.e. ~95% at 3*tau, so
         * "done within 10 s" means tau ~= 10/3 = 3.33 s; 3.0 s is the clean constant giving
         * 1 - e^(-10/3) ~= 96.4% at 10 s. This only governs the ordinary idle re-centring path
         * (yawIdle in GestureDetector::process(): no button, not moving, not scroll-active, below
         * the scroll start threshold). A misalignment large enough to engage scrolling never
         * reaches this EMA; that case is GestureDetector::STUCK_SCROLL_RECOVERY_S (30 s).
         * Tilt (DEFAULT_TILT_TIME_CONSTANT_S) is deliberately unchanged -- Edgar asked for Z only.
         */
        static constexpr float DEFAULT_YAW_TIME_CONSTANT_S{3.0f};

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
         * Number of plausible resting samples averaged together to seed the baseline,
         * instead of seeding from just one. At the nominal 50 Hz sample rate this is
         * ~10 s of accumulated quiet samples -- not necessarily 10 continuous seconds,
         * since implausible samples are skipped rather than counted (see the class doc
         * and isPlausibleRestingSample()). #1181, Edgar.
         */
        static constexpr int SETTLING_SAMPLES{500};

        /**
         * A resting board reads ~1 g in total. Anything outside [0.5 g, 1.5 g]
         * (in particular the all-zero reset value) is not a resting reading and
         * must not seed the baseline.
         */
        static constexpr float STANDARD_GRAVITY_MS2{9.81f};
        static constexpr float MIN_RESTING_GRAVITY_MS2{0.5f * STANDARD_GRAVITY_MS2};
        static constexpr float MAX_RESTING_GRAVITY_MS2{1.5f * STANDARD_GRAVITY_MS2};

        /**
         * Real hardware finding (#1181, Edgar's BLE/LED bring-up pass): a plausible gravity
         * MAGNITUDE alone is not enough to know the board is genuinely at rest -- it reads
         * ~1 g at almost any fixed orientation, including while still being held/rotated during
         * power-on or BLE pairing. Seeding mid-handling bakes in whatever angle the board
         * happened to be at that instant as "centre". If that angle differs from the board's
         * real resting stance by more than the movement/scroll START thresholds, the resulting
         * offset reads as permanent deliberate movement (by the same design that intentionally
         * keeps a genuine sustained lean from ever being absorbed, see adapt()/GestureDetector)
         * -- confirmed live: a board seeded while being handled sat stuck reporting a large
         * constant deltaX/deltaY (and an off-centre LED ball) indefinitely, with no automatic
         * recovery short of the BOOT-button re-zero. So seeding also requires the gyro to show
         * no rotation: the board must be truly motionless, not just oriented consistently with
         * gravity, at the moment it becomes "zero".
         */
        static constexpr float MAX_SEED_GYRO_DPS{5.0f};

        /**
         * Gyro-Z bias compensation for the yaw baseline (#1181, Edgar: "scroll should be based
         * on the absolute angle... on at abs(10+), off at abs(5)" -- not repeatable on hardware).
         *
         * The driver's yaw (Qmi8658C orientation.z) integrates raw gyro Z with no bias removal,
         * so at rest it ramps at the gyro's zero-rate offset -- measured live on the real board:
         * gyroZ = 0.5-0.6 dps at rest. Before this fix the yaw baseline only followed that ramp
         * through the idle EMA, and an EMA chasing a ramp of rate r lags it by exactly r * tau:
         * calibratedYaw sat at a constant ~+2.6 deg on a board nobody was touching (confirmed in
         * the live VERIFY log), shifting the 10/5 deg Schmitt trigger to ~7.4/2.4 deg one way and
         * ~12.6/7.6 deg the other. Worse, the baseline is (by design) frozen while scrolling, so
         * every engaged second added another ~0.55 deg: after a 10 s scroll the "off" point in
         * the drift direction had moved past the user's own starting position. Same hand
         * motion, different outcome, depending on direction and how long the previous scroll
         * lasted.
         *
         * Fix: estimate the zero-rate offset (deg/s) from the gyro Z of the same resting samples
         * that seed the baselines, then advance the yaw baseline along that drift EVERY sample
         * (scrolling or not), so calibratedYaw measures real rotation only. The estimate keeps
         * refining slowly (this time constant) only while yaw is idle AND the gyro is quiet
         * (|gyroZ| <= MAX_SEED_GYRO_DPS, the seeding criterion), to follow thermal bias drift.
         */
        static constexpr float YAW_RATE_BIAS_TIME_CONSTANT_S{30.0f};

        explicit Calibration(float tiltTimeConstantS = DEFAULT_TILT_TIME_CONSTANT_S,
                             float yawTimeConstantS = DEFAULT_YAW_TIME_CONSTANT_S,
                             int warmupSamples = DEFAULT_WARMUP_SAMPLES);

        /**
         * Record the latest raw reading. Once the warm-up has elapsed, accumulates
         * plausible resting samples (see isPlausibleRestingSample()) and seeds both
         * baselines from their average once SETTLING_SAMPLES have been collected.
         */
        void observe(const MotionSample &sample);

        /**
         * Adapt each baseline toward the latest raw reading by one EMA step of `dtSeconds`,
         * independently: tilt adapts when `tiltIdle`, yaw adapts when `yawIdle`. Does nothing
         * before seeding, regardless of either flag.
         *
         * Split into two independent flags purely so the caller can compute them separately
         * (#1181) -- behaviour is otherwise UNCHANGED from the original single shared `idle`
         * flag: each caller-computed condition is exactly what it was before, just no longer
         * ANDed into one bool before reaching here. See GestureDetector::process() and
         * snapYawToRaw() for the real fix to the drift/deadlock problem that motivated looking
         * at this code -- it does NOT live in adapt() (a gradual EMA correction here, at any
         * time constant, conflicts with the ticket's own acceptance criterion that a long
         * scroll must never be absorbed: there is no tau that is simultaneously "exactly zero
         * drift for the strict test_sustained_scroll_is_not_absorbed window" and "eventually
         * corrects a stuck wheel" -- confirmed by trying it, see git history for the discarded
         * attempt). The fix is a separate, explicit timeout in GestureDetector instead.
         */
        void adapt(float dtSeconds, bool tiltIdle, bool yawIdle);

        /**
         * Instantly set both baselines to the latest raw reading. Ignored before
         * seeding (the pending seed will take a real reading anyway).
         */
        void rezero();

        /**
         * Instantly set ONLY the yaw baseline to the latest raw reading -- the same snap
         * rezero() does, but scoped to yaw alone. Ignored before seeding.
         *
         * Used by GestureDetector's stuck-scroll recovery timeout (#1181, Edgar's real-
         * hardware drift find): gyro Z has no accelerometer-style absolute reference, so its
         * integration drifts unboundedly even at rest. If that drift alone ever pushes
         * calibratedYaw past the scroll engage threshold, the scroll wheel latches active and,
         * by design (adapt()'s yawIdle requires the wheel be inactive, protecting a real
         * sustained scroll from ever being absorbed -- see its own doc comment), yaw can never
         * adapt its way back out: a one-way ratchet. GestureDetector detects "wheel active,
         * nothing moving, gyro quiet" persisting far longer than any real gesture plausibly
         * would, and calls this once to break out, instead of a gradual correction (which has
         * no tau that is both negligible over a real multi-second hold and effective within a
         * reasonable recovery time -- see adapt()'s doc comment).
         */
        void snapYawToRaw();

        /** Current gyro-Z zero-rate offset estimate, deg/s (see YAW_RATE_BIAS_TIME_CONSTANT_S). */
        [[nodiscard]] float yawRateBiasDps() const { return yawRateBias; }

        [[nodiscard]] bool isSeeded() const { return seeded; }

        /** Number of observed samples that were rejected before seeding (warm-up + implausible). */
        [[nodiscard]] int samplesRejectedBeforeSeed() const { return rejectedBeforeSeed; }

        /** Plausible samples accumulated toward the SETTLING_SAMPLES seed average so far (0 if not yet settling). */
        [[nodiscard]] int settledSampleCount() const { return settleSampleCount; }

        /**
         * True if the accelerometer magnitude is finite and within the resting-gravity window
         * AND the gyro shows no rotation (see MAX_SEED_GYRO_DPS) -- i.e. the board is genuinely
         * motionless, not just held at some fixed, gravity-plausible angle.
         */
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

        // Settling (seed-averaging) state -- see observe().
        bool settling{false};
        int settleSampleCount{0};
        float settleSumTiltX{0};
        float settleSumTiltY{0};
        float settleSumYaw{0};
        float settleSumGyroZ{0};
        float settleSumT{0}; ///< Sum of (sample time - first settling sample time), seconds.
        uint32_t settleFirstMs{0};

        float tiltX{0};
        float tiltY{0};
        float yaw{0};

        float rawTiltX{0};
        float rawTiltY{0};
        float rawYaw{0};
        float rawGyroZ{0};

        float yawRateBias{0}; ///< deg/s, see YAW_RATE_BIAS_TIME_CONSTANT_S.
    };
} // namespace Hilton
