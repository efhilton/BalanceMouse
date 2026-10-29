#include <Calibration.hpp>
#include <cmath>

namespace Hilton {
    Calibration::Calibration(const float tiltTimeConstantS, const float yawTimeConstantS)
        : tiltTau(tiltTimeConstantS), yawTau(yawTimeConstantS) {
    }

    void Calibration::observe(const MotionSample &sample) {
        rawTiltX = sample.accel.x;
        rawTiltY = sample.accel.y;
        rawYaw = sample.yaw;
        if (!seeded) {
            rezero();
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
        tiltX = rawTiltX;
        tiltY = rawTiltY;
        yaw = rawYaw;
    }
} // namespace Hilton
