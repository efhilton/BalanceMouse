#pragma once

#include <MouseButtonStateMachine.hpp>
#include <cmath>
#include <cstdint>

namespace Hilton {
    class Mouse {
    public:
        using MovementDt = int8_t;
        using ScrollWheelDt = int8_t;

        static constexpr ScrollWheelDt ANGLE_THRESHOLD_START{16};
        static constexpr ScrollWheelDt ANGLE_THRESHOLD_STOP{5};
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
