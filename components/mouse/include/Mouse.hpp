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
        static constexpr MovementDt MOVEMENT_THRESHOLD_START{13};
        static constexpr MovementDt MOVEMENT_THRESHOLD_STOP{5};

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
