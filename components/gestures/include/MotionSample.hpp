#pragma once

#include <cstdint>

namespace Hilton {
    /**
     * Transport/driver-agnostic IMU sample consumed by GestureDetector.
     * Built from Qmi8658C::DataPoint on the device (see MouseEventDetector) and
     * synthesised directly by host tests.
     */
    struct MotionSample {
        struct Vec3 {
            float x{0};
            float y{0};
            float z{0};
        };

        Vec3 accel{}; ///< Linear acceleration in m/s^2 (Qmi8658C rawAccel).
        Vec3 gyro{}; ///< Angular rate in deg/s (Qmi8658C rawGyro).
        float yaw{0}; ///< Integrated yaw in degrees (Qmi8658C orientation.z).
        uint32_t timestampMs{0}; ///< Monotonic sample time in milliseconds.
    };
} // namespace Hilton
