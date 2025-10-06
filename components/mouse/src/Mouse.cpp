#include <cmath>
#include <cstdio>
#include <Mouse.hpp>
#include <MouseButtonStateMachine.hpp>
#include <stdexcept>
#include <string>

namespace Hilton {
    Mouse::Mouse()
        : leftButton(MouseButtonStateMachine::Button::Left)
          , rightButton(MouseButtonStateMachine::Button::Right)
          , middleButton(MouseButtonStateMachine::Button::Middle)
          , wheelButton(MouseButtonStateMachine::Button::Wheel) {
    }

    MouseButtonStateMachine &Mouse::getLeftButton() {
        return leftButton;
    }

    MouseButtonStateMachine &Mouse::getRightButton() {
        return rightButton;
    }

    MouseButtonStateMachine &Mouse::getMiddleButton() {
        return middleButton;
    }

    MouseButtonStateMachine &Mouse::getWheelButton() {
        return wheelButton;
    }

    MouseButtonStateMachine &Mouse::getButton(const MouseButtonStateMachine::Button &queryButton) {
        switch (queryButton) {
            case MouseButtonStateMachine::Button::Left:
                return leftButton;
            case MouseButtonStateMachine::Button::Right:
                return rightButton;
            case MouseButtonStateMachine::Button::Middle:
                return middleButton;
            case MouseButtonStateMachine::Button::Wheel:
                return wheelButton;
            default:
                throw std::invalid_argument("Invalid button type");
        }
    }

    void Mouse::updateMovement(const MovementDt dx, const MovementDt dy) {
        movement.deltaX = static_cast<MovementDt>(std::clamp(dx,
                                                             static_cast<MovementDt>(-127),
                                                             static_cast<MovementDt>(127)));
        movement.deltaY = static_cast<MovementDt>(std::clamp(dy,
                                                             static_cast<MovementDt>(-127),
                                                             static_cast<MovementDt>(127)));
        const auto spd = std::sqrt(movement.deltaX * movement.deltaX + movement.deltaY * movement.deltaY);
        if (!moving && spd > MOVEMENT_THRESHOLD_START) {
            moving = true;
        } else if (moving && spd < MOVEMENT_THRESHOLD_STOP) {
            moving = false;
        }
    }

    Mouse::Movement &Mouse::getMovement() {
        return movement;
    }

    bool Mouse::isMoving() const {
        return moving;
    }

    void Mouse::setAngle(const ScrollWheelDt value) {
        swAngle = static_cast<ScrollWheelDt>(std::clamp(static_cast<ScrollWheelDt>(value),
                                                        static_cast<ScrollWheelDt>(-127),
                                                        static_cast<ScrollWheelDt>(127)));
        if (!scrollWheelActive && std::abs(swAngle) > ANGLE_THRESHOLD_START) {
            // trigger on
            scrollWheelActive = true;
        } else if (scrollWheelActive && std::abs(swAngle) < ANGLE_THRESHOLD_STOP) {
            // trigger off
            scrollWheelActive = false;
        }
    }

    Mouse::ScrollWheelDt Mouse::getAngle() const {
        return swAngle;
    }

    void Mouse::debugState() const {
        printf(
            "Left: %9s Right: %9s Middle: %9s deltaX=%3d, deltaY=%3d angle: %3d\n",
            leftButton.getStateName().c_str(),
            rightButton.getStateName().c_str(),
            middleButton.getStateName().c_str(),
            movement.deltaX,
            movement.deltaY,
            swAngle);
    }

    bool Mouse::isScrollWheelActive() const {
        return scrollWheelActive;
    }

    bool Mouse::isButtonPressed(const MouseButtonStateMachine::Button &queryButton) {
        const MouseButtonStateMachine &button = getButton(queryButton);
        return button.getState() == MouseButtonStateMachine::State::Pressed;
    }

    bool Mouse::isButtonIdle(const MouseButtonStateMachine::Button &queryButton) {
        const MouseButtonStateMachine &button = getButton(queryButton);
        return button.getState() == MouseButtonStateMachine::State::Idle;
    }

    void Mouse::printStatus() const {
        printf("L: %9s, R: %9s, M: %9s, W: %9s, X=%4d, Y=%4d, ScrollWheel=%4d (%s)\n",
               leftButton.getStateName().c_str(),
               rightButton.getStateName().c_str(),
               middleButton.getStateName().c_str(),
               wheelButton.getStateName().c_str(),
               movement.deltaX,
               movement.deltaY,
               swAngle,
               isScrollWheelActive() ? "ON" : "OFF");
    }
}
