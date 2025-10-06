#pragma once

#include "MouseButtonStateMachine.hpp"
#include <cstdint>
#include <string>

namespace Hilton {

class Mouse
{
public:
    using MovementDt = int8_t;
    using ScrollWheelDt = int8_t;

    enum MouseButtonMask : EventBits_t {
        None = 0,
        LeftButton = BIT0,
        RightButton = BIT1,
        MiddleButton = BIT2,
        WheelButton = BIT3
    };

    static constexpr ScrollWheelDt SCROLLWHEEL_THRESHOLD{10};
    static constexpr MovementDt MOVEMENT_THRESHOLD{10};

    struct Movement
    {
        MovementDt deltaX;
        MovementDt deltaY;
    };

    explicit Mouse();

    MouseButtonStateMachine &getLeftButton();
    MouseButtonStateMachine &getRightButton();
    MouseButtonStateMachine &getMiddleButton();
    MouseButtonStateMachine &getWheelButton();
    MouseButtonStateMachine &getButton(const MouseButtonStateMachine::Button &queryButton);

    void updateMovement(MovementDt dx, MovementDt dy);
    [[nodiscard]] Movement &getMovement();
    [[nodiscard]] bool isMoving() const;

    void setScrollWheel(ScrollWheelDt value);
    [[nodiscard]] ScrollWheelDt getScrollWheel() const;
    [[nodiscard]] bool isScrollWheelActive() const;

    void debugState() const;

    bool isButtonPressed(const MouseButtonStateMachine::Button &queryButton);
    bool isButtonIdle(const MouseButtonStateMachine::Button &queryButton);
    bool isButtonReleased(const MouseButtonStateMachine::Button &queryButton);

private:
    MouseButtonStateMachine leftButton;
    MouseButtonStateMachine rightButton;
    MouseButtonStateMachine middleButton;
    MouseButtonStateMachine wheelButton;
    Movement movement{0, 0};
    ScrollWheelDt scrollWheel{0};
};

} // namespace Hilton
