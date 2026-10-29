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
        // NaN fails both comparisons, so a NaN/inf reading is rejected too.
        return std::isfinite(sample.yaw)
               && magnitude >= MIN_RESTING_GRAVITY_MS2
               && magnitude <= MAX_RESTING_GRAVITY_MS2;
    }

    void Calibration::observe(const MotionSample &sample) {
        rawTiltX = sample.accel.x;
        rawTiltY = sample.accel.y;
        rawYaw = sample.yaw;
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
        snapToRaw();
        seeded = true;
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

    void Calibration::adapt(const float dtSeconds, const bool idle) {
        if (!idle || !seeded) {
            return;
        }
        const float aTilt = emaAlpha(dtSeconds, tiltTau);
        const float aYaw = emaAlpha(dtSeconds, yawTau);
        tiltX += aTilt * (rawTiltX - tiltX);
        tiltY += aTilt * (rawTiltY - tiltY);
        yaw += aYaw * (rawYaw - yaw);
    }

    void Calibration::rezero() {
        if (seeded) {
            snapToRaw();
        }
    }

    void Calibration::snapToRaw() {
        tiltX = rawTiltX;
        tiltY = rawTiltY;
        yaw = rawYaw;
    }
} // namespace Hilton
