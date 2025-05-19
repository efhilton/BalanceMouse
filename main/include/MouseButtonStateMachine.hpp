#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <string>

namespace Hilton {

class MouseButtonStateMachine
{
public:
    enum class State { Idle, Pressed, Clicked, DoubleClicked, Dragging };
    enum class Event { MouseDown, MouseUp, MouseMove, TimerEvent };

    MouseButtonStateMachine(const std::string &myName)
        : state(State::Idle)
        , name(myName)
    {
        // Create a FreeRTOS timer for this button
        timer = xTimerCreate(
            ("Timer_" + name).c_str(),           // Timer name
            pdMS_TO_TICKS(doubleClickThreshold), // Timer period
            pdFALSE,                             // One-shot timer
            this,                                // Timer ID (pass this instance)
            timerCallback                        // Callback function
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

    void handleEvent(const Event event, const float x = 0, const float y = 0)
    {
        switch (state) {
        case State::Idle:
            if (event == Event::MouseDown) {
                state = State::Pressed;
                startX = x;
                startY = y;
                std::cout << name << ": Pressed\n";
            }
            break;

        case State::Pressed:
            if (event == Event::MouseMove) {
                if (distance(startX, startY, x, y) > dragThreshold) {
                    state = State::Dragging;
                    std::cout << name << ": Dragging\n";
                }
            } else if (event == Event::MouseUp) {
                const auto now = xTaskGetTickCount();
                if ((now - lastClickTime) <= pdMS_TO_TICKS(doubleClickThreshold)) {
                    state = State::DoubleClicked;
                    std::cout << name << ": DoubleClicked\n";
                    startTimer(doubleClickTimeout); // Timer for DoubleClicked state
                } else {
                    state = State::Clicked;
                    std::cout << name << ": Clicked\n";
                    startTimer(doubleClickThreshold); // Timer for possible second click
                }
                lastClickTime = now;
            }
            break;

        case State::Dragging:
            if (event == Event::MouseUp) {
                state = State::Idle;
                std::cout << name << ": Idle\n";
            }
            break;

        case State::Clicked:
        case State::DoubleClicked:
            if (event == Event::TimerEvent) {
                state = State::Idle;
                std::cout << name << ": Idle\n";
            } else if (event == Event::MouseDown) {
                state = State::Pressed;
                startX = x;
                startY = y;
                std::cout << name << ": Pressed\n";
            }
            break;
        }
    }

    State getState() const { return state; }

private:
    State state;
    float startX = 0, startY = 0;          // For detecting dragging
    const float dragThreshold = 20.0f;     // Pixel threshold for drag detection
    const int doubleClickThreshold = 1500; // Max time (ms) between clicks for double click
    const int doubleClickTimeout = 200;    // Timeout for DoubleClicked state

    TickType_t lastClickTime = ~0;  // Track the last click time (in ticks)
    TimerHandle_t timer = nullptr; // FreeRTOS timer handle
    std::string name;

    // Start the timer
    void startTimer(const int timeout)
    {
        if (!timer) {
            return;
        }
        if (xTimerIsTimerActive(timer)) {
            xTimerStop(timer, 0);
        }
        if (xTimerChangePeriod(timer, pdMS_TO_TICKS(timeout), 0) != pdPASS) {
            std::cerr << name << ": Failed to start timer!\n";
        }
        if (xTimerStart(timer, 0) != pdPASS) { // Restart the timer explicitly
            std::cerr << name << ": Failed to restart timer!\n";
        }
    }

    // FreeRTOS timer callback (static because FreeRTOS timers require static callbacks)
    static void timerCallback(TimerHandle_t xTimer)
    {
        auto *instance = static_cast<MouseButtonStateMachine *>(pvTimerGetTimerID(xTimer));
        if (instance) {
            instance->handleEvent(Event::TimerEvent);
        }
    }

    // Helper function to calculate distance between two points
    float distance(const float x1, const float y1, const float x2, const float y2)
    {
        return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
};
} // namespace Hilton