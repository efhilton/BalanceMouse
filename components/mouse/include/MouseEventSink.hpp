#pragma once

#include <MouseButtonStateMachine.hpp>
#include <cstdint>
#include <vector>

namespace Hilton {
    /**
     * The seam between gesture detection and everything that consumes mouse
     * gestures (BLE HID transport, LED feedback, logging, host-side tests).
     *
     * GestureDetector emits discrete events into a sink; it knows nothing about
     * the transport. Implementations must be cheap and non-blocking: they are
     * invoked from the IMU sampling task at ~50Hz.
     */
    class MouseEventSink {
    public:
        using Button = MouseButtonStateMachine::Button;

        virtual ~MouseEventSink() = default;

        virtual void onButtonDown(Button button) = 0;

        virtual void onButtonUp(Button button) = 0;

        /** Relative cursor motion for this sample; HID convention (+x right, +y down). */
        virtual void onMovement(int8_t dx, int8_t dy) = 0;

        /** Whole wheel ticks for this sample; HID convention (+ = scroll up / away from user). */
        virtual void onScroll(int8_t ticks) = 0;
    };

    /** Forwards every event to several sinks, in registration order. Not thread-safe to mutate while in use. */
    class MouseEventFanout final : public MouseEventSink {
    public:
        void addSink(MouseEventSink *sink) {
            if (sink != nullptr) {
                sinks.push_back(sink);
            }
        }

        void onButtonDown(const Button button) override {
            for (auto *s: sinks) s->onButtonDown(button);
        }

        void onButtonUp(const Button button) override {
            for (auto *s: sinks) s->onButtonUp(button);
        }

        void onMovement(const int8_t dx, const int8_t dy) override {
            for (auto *s: sinks) s->onMovement(dx, dy);
        }

        void onScroll(const int8_t ticks) override {
            for (auto *s: sinks) s->onScroll(ticks);
        }

    private:
        std::vector<MouseEventSink *> sinks;
    };
} // namespace Hilton
