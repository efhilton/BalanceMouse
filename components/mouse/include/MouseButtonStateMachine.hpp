#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <iostream>
#include <string>

namespace Hilton {

class MouseButtonStateMachine
{
public:
    enum class Button { Left, Right, Middle, Wheel };
    enum class State { Idle, Pressed, Released };
    enum class Event { MouseDown, MouseUp };

    explicit MouseButtonStateMachine(const Button &button)
        : state(State::Idle)
        , lastTick(0)
        , button(button)
    {
    }

    void handleEvent(const Event event)
    {
        TickType_t currentTick = xTaskGetTickCount();
        switch (state) {
        case State::Idle:
            if (event == Event::MouseDown) {
                state = State::Pressed;
            }
            break;

        case State::Pressed:
            if (event == Event::MouseUp) {
                state = State::Released;
                lastTick = currentTick + pdMS_TO_TICKS(clickTimeout);
            }
            break;

        case State::Released:
            if (currentTick >= lastTick) {
                state = State::Idle;
            }
            break;
        }
    }

    [[nodiscard]] State getState() const { return state; }
    [[nodiscard]] std::string getStateName() const
    {
        switch (state) {
        case State::Idle:
            return "Idle";
        case State::Pressed:
            return "Pressed";
        case State::Released:
            return "Released";
        default:
            return "Unknown";
        }
    }

    [[nodiscard]] std::string getButtonName() const
    {
        switch (button) {
        case Button::Left:
            return "Left";
        case Button::Right:
            return "Right";
        case Button::Middle:
            return "Middle";
        default:
            return "Unknown";
        }
    }

private:
    State state;
    const int clickTimeout = 200;  // Timeout for transitioning back to Idle
    TickType_t lastTick;          // Last recorded tick for timeout comparison
    const Button &button;
};

} // namespace Hilton