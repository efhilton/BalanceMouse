#pragma once

#include <MouseButtonStateMachine.hpp>

namespace Hilton {
    class Mouse {
    public:
        using MovementDt = int8_t;
        using ScrollWheelDt = int8_t;

        static constexpr ScrollWheelDt ANGLE_THRESHOLD_START{16};
        static constexpr ScrollWheelDt ANGLE_THRESHOLD_STOP{5};
        static constexpr MovementDt MOVEMENT_THRESHOLD_START{13};
        static constexpr MovementDt MOVEMENT_THRESHOLD_STOP{5};

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
