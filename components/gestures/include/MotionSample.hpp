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

        /**
         * Linear acceleration in m/s^2, in CURSOR-channel axes, not raw sensor axes.
         *
         * Axis convention is Edgar's authoritative one (#1181, 2026-11-02, replaces the
         * ambiguous "roll/pitch" language used earlier in this ticket): right-hand rule,
         * X = right, Y = forward, Z = up.
         *   accel.x: left/right cursor channel. Responds to rotation about Y (forward axis).
         *   accel.y: up/down cursor channel. Responds to rotation about X (right axis).
         * MouseEventDetector::onSensorDataIsReady() maps Qmi8658C::DataPoint's rawAccel into
         * these before building a MotionSample -- see the comment there for why that mapping
         * is not a straight-through copy of the chip's raw x/y (the QMI8658 is mounted with
         * its own x/y swapped relative to the board's logical X/Y, a real-hardware finding,
         * #1181, Edgar).
         */
        Vec3 accel{};
        Vec3 gyro{}; ///< Angular rate in deg/s (Qmi8658C rawGyro).
        float yaw{0}; ///< Integrated yaw in degrees (Qmi8658C orientation.z).
        uint32_t timestampMs{0}; ///< Monotonic sample time in milliseconds.
    };
} // namespace Hilton
