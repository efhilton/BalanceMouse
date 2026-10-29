#pragma once

#include <string>

namespace Hilton {

class MouseButtonStateMachine
{
public:
    enum class Button { Left, Right, Middle };
    enum class State { Idle, Pressed };
    enum class Event { MouseDown, MouseUp };

    // NOTE: `button` is taken and stored BY VALUE. It used to be stored as a
    // `const Button &`, but Mouse's constructor passes temporaries
    // (e.g. `MouseButtonStateMachine::Button::Left`), so the stored reference
    // dangled as soon as the constructor returned.
    explicit MouseButtonStateMachine(const Button button)
        : state(State::Idle)
        , button(button)
    {
    }

    void handleEvent(const Event event) {
        switch (state) {
            case State::Idle:
                if (event == Event::MouseDown) {
                    state = State::Pressed;
                }
                break;

            case State::Pressed:
                if (event == Event::MouseUp) {
                    state = State::Idle;
                }
                break;
        }
    }

    [[nodiscard]] State getState() const { return state; }
    [[nodiscard]] Button getButton() const { return button; }
    [[nodiscard]] std::string getStateName() const
    {
        switch (state) {
        case State::Idle:
            return "Idle";
        case State::Pressed:
            return "Pressed";
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
    Button button;
};

} // namespace Hilton
