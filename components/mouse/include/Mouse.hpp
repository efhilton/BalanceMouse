#pragma once

#include <MouseButtonStateMachine.hpp>
#include <cmath>
#include <cstdint>

namespace Hilton {
    class Mouse {
    public:
        using MovementDt = int8_t;
        using ScrollWheelDt = int8_t;

        /**
         * Scroll-wheel engage/disengage, Schmitt trigger (#1181, Edgar): engage only once
         * abs(angle) truly exceeds GestureDetector::SCROLL_ENGAGE_DEG (10 deg), stay engaged
         * until it drops back to about GestureDetector::SCROLL_DISENGAGE_DEG (5 deg) -- a real
         * gap, not one threshold, so a reading sitting right at the edge can't chatter on/off.
         * Mouse has no notion of degrees (it only ever sees the already-scaled report-unit
         * angle), so these are the report-unit equivalents given
         * GestureDetector::MAX_SCROLL_ANGLE_DEG = 80 (SCROLL_SCALE = 127/80 = 1.5875 units/deg):
         *   START = 15 -> engages at report 16, i.e. angle > 15/1.5875 = 10.08 deg (~10).
         *   STOP  = 8  -> disengages at report 7, i.e. angle < 8/1.5875 = 5.04 deg (~5).
         * If MAX_SCROLL_ANGLE_DEG is retuned, these must be recomputed to match --
         * test_scroll_engage_disengage_thresholds_match_mouse_hysteresis (host_test) fails if
         * they drift apart.
         */
        static constexpr ScrollWheelDt ANGLE_THRESHOLD_START{15};
        static constexpr ScrollWheelDt ANGLE_THRESHOLD_STOP{8};
        /**
         * Cursor-movement start/stop hysteresis (the XY centre deadzone), compared against
         * spd = hypot(dx, dy) of the ALREADY-curved-and-scaled HID deltas
         * (GestureDetector::movementDeltaFromTilt: delta = round(n^2 * 127 * 0.5), n = |tilt|
         * normalised to MOVEMENT_FULL_SCALE_TILT_DEG = 20 deg, full output 63.5). Mouse has no
         * notion of degrees, so these are the report-unit equivalents of the intended angles.
         * #1181, Edgar: "the deadzone for xy movement is very large" -- the old 13/5 were tuned
         * for the unscaled curve (full output 127) and were not rescaled when
         * MOVEMENT_SPEED_SCALE = 0.5 halved it, so the start angle grew from ~6.4 to ~9 deg.
         * Re-derived for the 0.5 scale (n * 20 deg, small-angle approximation):
         *   START = 6 -> n = sqrt(6/63.5) = 0.307 -> ~6.15 deg (old 13 @ full 127: n = 0.320,
         *                ~6.4 deg; at the 0.5 scale 13 had become ~9 deg).
         *   STOP  = 2 -> n = sqrt(2/63.5) = 0.177 -> ~3.5 deg (old 5 @ full 127: n = 0.198,
         *                ~4.0 deg; at the 0.5 scale 5 had become ~5.6 deg).
         * (With rounding, the actual single-axis crossings are delta >= 7, i.e. tilt >= ~6.3 deg,
         * to start and delta <= 1, i.e. tilt < ~3.0 deg, to stop.)
         * Must be re-derived if GestureDetector::MOVEMENT_SPEED_SCALE, MOVEMENT_ACCEL_EXPONENT
         * or MOVEMENT_FULL_SCALE_TILT_DEG change --
         * test_movement_start_stop_thresholds_match_intended_deadzone (host_test) fails if the
         * curve and these thresholds drift apart. Independent of the calibration idle gate,
         * which has its own GestureDetector::TILT_IDLE_MAGNITUDE_THRESHOLD.
         */
        static constexpr MovementDt MOVEMENT_THRESHOLD_START{6};
        static constexpr MovementDt MOVEMENT_THRESHOLD_STOP{2};

        // Valid range of a signed 8-bit HID mouse report axis/wheel value.
        static constexpr float REPORT_MIN{-127.0f};
        static constexpr float REPORT_MAX{127.0f};

        /**
         * Converts a scaled float into the int8_t report range by clamping to
         * [-127, 127] BEFORE the narrowing cast. Casting first (the old
         * behaviour) is undefined/implementation-defined for out-of-range
         * values and in practice wrapped around, so a hard tilt past ~85% of
         * the accelerometer's +/-2G full scale flipped the cursor direction.
         * Truncates toward zero (same as the old static_cast for in-range
         * values). NaN maps to 0.
         */
        static int8_t saturateToReport(const float value) {
            if (std::isnan(value)) {
                return 0;
            }
            const float clamped = value < REPORT_MIN ? REPORT_MIN : (value > REPORT_MAX ? REPORT_MAX : value);
            return static_cast<int8_t>(clamped);
        }

        struct Movement {
            MovementDt deltaX;
            MovementDt deltaY;
        };

        explicit Mouse();

        MouseButtonStateMachine &getLeftButton();

        MouseButtonStateMachine &getRightButton();

        MouseButtonStateMachine &getMiddleButton();

        MouseButtonStateMachine &getButton(const MouseButtonStateMachine::Button &queryButton);

        void updateMovement(MovementDt dx, MovementDt dy);

        [[nodiscard]] Movement &getMovement();

        [[nodiscard]] bool isMoving() const;

        void setAngle(ScrollWheelDt value);

        [[nodiscard]] ScrollWheelDt getAngle() const;

        [[nodiscard]] bool isScrollWheelActive() const;

        void debugState() const;

        bool isButtonPressed(const MouseButtonStateMachine::Button &queryButton);

        bool isButtonIdle(const MouseButtonStateMachine::Button &queryButton);

        void printStatus() const;

    private:
        MouseButtonStateMachine leftButton;
        MouseButtonStateMachine rightButton;
        MouseButtonStateMachine middleButton;
        Movement movement{0, 0};
        ScrollWheelDt swAngle{0};
        bool scrollWheelActive{false};
        bool moving{false};
    };
} // namespace Hilton
