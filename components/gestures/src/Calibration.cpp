#include <Calibration.hpp>
#include <cmath>

namespace Hilton {
    Calibration::Calibration(const float tiltTimeConstantS, const float yawTimeConstantS, const int warmupSamples)
        : tiltTau(tiltTimeConstantS), yawTau(yawTimeConstantS), warmupRemaining(warmupSamples > 0 ? warmupSamples : 0) {
    }

    bool Calibration::isPlausibleRestingSample(const MotionSample &sample) {
        const float magnitude = std::sqrt(sample.accel.x * sample.accel.x
                                          + sample.accel.y * sample.accel.y
                                          + sample.accel.z * sample.accel.z);
        const float gyroMagnitude = std::sqrt(sample.gyro.x * sample.gyro.x
                                              + sample.gyro.y * sample.gyro.y
                                              + sample.gyro.z * sample.gyro.z);
        // NaN fails every comparison, so a NaN/inf reading (accel or gyro) is rejected too.
        return std::isfinite(sample.yaw)
               && magnitude >= MIN_RESTING_GRAVITY_MS2
               && magnitude <= MAX_RESTING_GRAVITY_MS2
               && gyroMagnitude <= MAX_SEED_GYRO_DPS;
    }

    void Calibration::observe(const MotionSample &sample) {
        rawTiltX = sample.accel.x;
        rawTiltY = sample.accel.y;
        rawYaw = sample.yaw;
        rawGyroZ = sample.gyro.z;
        if (seeded) {
            return;
        }
        if (warmupRemaining > 0) {
            --warmupRemaining;
            ++rejectedBeforeSeed;
            return;
        }
        if (!isPlausibleRestingSample(sample)) {
            ++rejectedBeforeSeed;
            return;
        }
        // Average SETTLING_SAMPLES plausible readings instead of seeding from just one
        // (#1181, Edgar). Only plausible samples ever reach here, so a rejected one above
        // simply doesn't count -- intermittent jostling makes settling take a little
        // longer, it doesn't reset progress or fail outright.
        if (!settling) {
            settling = true;
            settleSampleCount = 0;
            settleSumTiltX = 0.0f;
            settleSumTiltY = 0.0f;
            settleSumYaw = 0.0f;
            settleSumGyroZ = 0.0f;
            settleSumT = 0.0f;
            settleFirstMs = sample.timestampMs;
        }
        const float tRel = static_cast<float>(sample.timestampMs - settleFirstMs) / 1000.0f;
        ++settleSampleCount;
        settleSumTiltX += sample.accel.x;
        settleSumTiltY += sample.accel.y;
        settleSumYaw += sample.yaw;
        settleSumGyroZ += sample.gyro.z;
        settleSumT += tRel;
        if (settleSampleCount >= SETTLING_SAMPLES) {
            const float n = static_cast<float>(settleSampleCount);
            tiltX = settleSumTiltX / n;
            tiltY = settleSumTiltY / n;
            // Gyro-Z zero-rate offset (see YAW_RATE_BIAS_TIME_CONSTANT_S). The averaged yaw is
            // where the drifting raw yaw was at the window's mean time, not now: carry it
            // forward along the drift to this sample, or a ~10 s window would start the
            // baseline bias * ~5 s behind (~2.6 deg on the real board).
            yawRateBias = settleSumGyroZ / n;
            yaw = settleSumYaw / n + yawRateBias * (tRel - settleSumT / n);
            seeded = true;
        }
    }

    float Calibration::emaAlpha(const float dtSeconds, const float timeConstantS) {
        if (!(dtSeconds > 0.0f)) {
            return 0.0f;
        }
        if (!(timeConstantS > 0.0f)) {
            return 1.0f;
        }
        // Exact discretisation of a first-order low-pass: independent of sample rate jitter.
        return 1.0f - std::exp(-dtSeconds / timeConstantS);
    }

    void Calibration::adapt(const float dtSeconds, const bool tiltIdle, const bool yawIdle) {
        if (!seeded) {
            return;
        }
        if (tiltIdle) {
            const float aTilt = emaAlpha(dtSeconds, tiltTau);
            tiltX += aTilt * (rawTiltX - tiltX);
            tiltY += aTilt * (rawTiltY - tiltY);
        }
        // Follow the gyro-Z bias drift of the raw yaw every sample, idle or not (see
        // YAW_RATE_BIAS_TIME_CONSTANT_S): this is the sensor's zero-rate ramp, not a gesture,
        // so it must not accumulate into calibratedYaw -- in particular not while scrolling,
        // when the EMA below is (correctly) frozen.
        if (dtSeconds > 0.0f) {
            yaw += yawRateBias * dtSeconds;
        }
        if (yawIdle) {
            const float aYaw = emaAlpha(dtSeconds, yawTau);
            yaw += aYaw * (rawYaw - yaw);
            if (std::abs(rawGyroZ) <= MAX_SEED_GYRO_DPS) {
                yawRateBias += emaAlpha(dtSeconds, YAW_RATE_BIAS_TIME_CONSTANT_S) * (rawGyroZ - yawRateBias);
            }
        }
    }

    void Calibration::rezero() {
        if (seeded) {
            snapToRaw();
        }
    }

    void Calibration::snapYawToRaw() {
        if (seeded) {
            yaw = rawYaw;
        }
    }

    void Calibration::snapToRaw() {
        tiltX = rawTiltX;
        tiltY = rawTiltY;
        yaw = rawYaw;
    }
} // namespace Hilton
