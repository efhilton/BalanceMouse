#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <utility>

namespace Hilton {

class MouseButtonStateMachine
{
public:
    enum class State { Idle, Pressed, Released };
    enum class Event { MouseDown, MouseUp, TimerEvent };

    explicit MouseButtonStateMachine(std::string myName)
        : state(State::Idle)
        , name(std::move(myName))
    {
        // Create a FreeRTOS timer for this button
        timer = xTimerCreate(
            ("Timer_" + name).c_str(),   // Timer name
            pdMS_TO_TICKS(clickTimeout), // Timer period
            pdFALSE,                     // One-shot timer
            this,                        // Timer ID (pass this instance)
            timerCallback                // Callback function
        );

        if (!timer) {
            std::cerr << name << ": Failed to create timer!\n";
        }
    }

    ~MouseButtonStateMachine()
    {
        if (timer) {
            xTimerDelete(timer, 0);
        }
    }

    void handleEvent(const Event event)
    {
        switch (state) {
        case State::Idle:
            if (event == Event::MouseDown) {
                state = State::Pressed;
            }
            break;

        case State::Pressed:
            if (event == Event::MouseUp) {
                state = State::Released;
                startTimer(clickTimeout);
            }
            break;

        case State::Released:
            if (event == Event::TimerEvent) {
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

private:
    State state;
    const int clickTimeout = 200;  // Timeout for DoubleClicked state
    TimerHandle_t timer = nullptr; // FreeRTOS timer handle
    std::string name;

    // Start the timer
    void startTimer(const int timeout) const
    {
        if (xTimerStart(timer, 0) != pdPASS) { // Restart the timer explicitly
            std::cerr << name << ": Failed to restart timer!\n";
        }
    }

    // FreeRTOS timer callback (static because FreeRTOS timers require static callbacks)
    static void timerCallback(TimerHandle_t xTimer)
    {
        const auto instance = static_cast<MouseButtonStateMachine *>(pvTimerGetTimerID(xTimer));
        if (!instance)
            return;
        instance->handleEvent(Event::TimerEvent);
    }
};
} // namespace Hilton