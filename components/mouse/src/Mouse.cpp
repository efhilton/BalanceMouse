#include "Mouse.hpp"
#include "MouseButtonStateMachine.hpp"
#include <cmath>

#include <cstdio>

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
        movement.deltaX = dx;
        movement.deltaY = dy;
    }

    Mouse::Movement &Mouse::getMovement() {
        return movement;
    }

    bool Mouse::isMoving() const {
        return std::sqrt(movement.deltaX * movement.deltaX + movement.deltaY * movement.deltaY)
               > MOVEMENT_THRESHOLD;
    }

    void Mouse::setScrollWheel(const ScrollWheelDt value) {
        scrollWheel = value;
    }

    Mouse::ScrollWheelDt Mouse::getScrollWheel() const {
        return scrollWheel;
    }

    void Mouse::debugState() const {
        printf(
            "Left: %9s Right: %9s Middle: %9s deltaX=%3d, deltaY=%3d ScrollWheel: %3d\n",
            leftButton.getStateName().c_str(),
            rightButton.getStateName().c_str(),
            middleButton.getStateName().c_str(),
            movement.deltaX,
            movement.deltaY,
            scrollWheel);
    }

    bool Mouse::isScrollWheelActive() const {
        return (std::abs(scrollWheel) > SCROLLWHEEL_THRESHOLD);
    }

    bool Mouse::isButtonPressed(const MouseButtonStateMachine::Button &queryButton) {
        const MouseButtonStateMachine &button = getButton(queryButton);
        return button.getState() == MouseButtonStateMachine::State::Pressed;
    }

    bool Mouse::isButtonIdle(const MouseButtonStateMachine::Button &queryButton) {
        const MouseButtonStateMachine &button = getButton(queryButton);
        return button.getState() == MouseButtonStateMachine::State::Idle;
    }

    bool Mouse::isButtonReleased(const MouseButtonStateMachine::Button &queryButton) {
        const MouseButtonStateMachine &button = getButton(queryButton);
        return button.getState() == MouseButtonStateMachine::State::Released;
    }

    void Mouse::printStatus() const {
        printf("Left: %s, Right: %s, Middle: %s, Wheel: %s, X=%d, Y=%d, ScrollWheel=%d\n",
               leftButton.getStateName().c_str(),
               rightButton.getStateName().c_str(),
               middleButton.getStateName().c_str(),
               wheelButton.getStateName().c_str(),
               movement.deltaX,
               movement.deltaY,
               scrollWheel);
    }
}
