#pragma once

// Shared fakes for the host tests: a recording MouseEventSink and an IMU sample generator.

#include <GestureDetector.hpp>
#include <MotionSample.hpp>
#include <MouseEventSink.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace TestSupport {
    using Hilton::MouseButtonStateMachine;
    using Button = MouseButtonStateMachine::Button;

    struct RecordedEvent {
        enum class Type { Down, Up, Move, Scroll } type;
        Button button{Button::Left};
        int dx{0};
        int dy{0};
        int ticks{0};
    };

    /** Fake sink: records everything, no hardware/BLE involved. */
    class RecordingSink final : public Hilton::MouseEventSink {
    public:
        void onButtonDown(const Button b) override { events.push_back({RecordedEvent::Type::Down, b}); }
        void onButtonUp(const Button b) override { events.push_back({RecordedEvent::Type::Up, b}); }

        void onMovement(const int8_t dx, const int8_t dy) override {
            events.push_back({RecordedEvent::Type::Move, Button::Left, dx, dy});
        }

        void onScroll(const int8_t t) override { events.push_back({RecordedEvent::Type::Scroll, Button::Left, 0, 0, t}); }

        [[nodiscard]] int count(const RecordedEvent::Type t) const {
            int n = 0;
            for (const auto &e: events) n += e.type == t ? 1 : 0;
            return n;
        }

        [[nodiscard]] int count(const RecordedEvent::Type t, const Button b) const {
            int n = 0;
            for (const auto &e: events) n += (e.type == t && e.button == b) ? 1 : 0;
            return n;
        }

        [[nodiscard]] int scrollTotal() const {
            int n = 0;
            for (const auto &e: events) n += e.type == RecordedEvent::Type::Scroll ? e.ticks : 0;
            return n;
        }

        void clear() { events.clear(); }

        std::vector<RecordedEvent> events;
    };

    /** Drives a GestureDetector with synthetic 50 Hz samples. */
    class Rig {
    public:
        static constexpr uint32_t PERIOD_MS{20};

        Hilton::Mouse mouse;
        Hilton::Calibration calibration;
        RecordingSink sink;
        Hilton::GestureDetector detector{mouse, calibration, &sink};
        uint32_t nowMs{1000};

        // Current "physical" state of the board.
        float ax{0}, ay{0}, az{9.8f};
        float gx{0}, gy{0}, gz{0};
        float yaw{0};

        bool step() {
            Hilton::MotionSample s{};
            s.accel = {ax, ay, az};
            s.gyro = {gx, gy, gz};
            s.yaw = yaw;
            s.timestampMs = nowMs;
            const bool handled = detector.process(s);
            nowMs += PERIOD_MS;
            return handled;
        }

        void run(const uint32_t durationMs) {
            for (uint32_t t = 0; t < durationMs; t += PERIOD_MS) step();
        }

        /** One sample with a gyro spike, then gyro back to rest. */
        void spike(const float gyroX, const float gyroY) {
            gx = gyroX;
            gy = gyroY;
            step();
            gx = 0;
            gy = 0;
        }
    };
} // namespace TestSupport
