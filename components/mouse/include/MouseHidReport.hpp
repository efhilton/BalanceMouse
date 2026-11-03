#pragma once

#include <MouseEventSink.hpp>
#include <array>
#include <cstdint>
#include <functional>

namespace Hilton {
    /**
     * Standard 4-byte boot-mouse-compatible input report:
     *   byte 0: button bitmask (bit0 = left, bit1 = right, bit2 = middle)
     *   byte 1: signed 8-bit X delta
     *   byte 2: signed 8-bit Y delta
     *   byte 3: signed 8-bit wheel delta
     *
     * Matches the report map in BleHidMouse.cpp (adapted from ESP-IDF's
     * examples/bluetooth/esp_hid_device mouse report map, no report ID).
     */
    using MouseHidReport = std::array<uint8_t, 4>;

    namespace HidButtonBits {
        static constexpr uint8_t LEFT{0x01};
        static constexpr uint8_t RIGHT{0x02};
        static constexpr uint8_t MIDDLE{0x04};
    }

    inline uint8_t hidButtonBit(const MouseButtonStateMachine::Button button) {
        switch (button) {
            case MouseButtonStateMachine::Button::Left:
                return HidButtonBits::LEFT;
            case MouseButtonStateMachine::Button::Right:
                return HidButtonBits::RIGHT;
            case MouseButtonStateMachine::Button::Middle:
                return HidButtonBits::MIDDLE;
        }
        return 0;
    }

    inline MouseHidReport makeMouseHidReport(const uint8_t buttons, const int8_t dx, const int8_t dy, const int8_t wheel) {
        return {
            static_cast<uint8_t>(buttons & 0x07),
            static_cast<uint8_t>(dx),
            static_cast<uint8_t>(dy),
            static_cast<uint8_t>(wheel),
        };
    }

    /**
     * Transport-agnostic MouseEventSink that turns discrete events into HID
     * reports and hands each one to `transmit`. Every button down/up, movement
     * delta and scroll tick produces exactly one report; the current button
     * mask is carried in every report so a host never sees a spurious release.
     */
    class MouseHidReportBuilder : public MouseEventSink {
    public:
        using Transmit = std::function<void(const MouseHidReport &)>;

        explicit MouseHidReportBuilder(Transmit transmit) : transmit(std::move(transmit)) {
        }

        void onButtonDown(const Button button) override {
            buttons = static_cast<uint8_t>(buttons | hidButtonBit(button));
            send(0, 0, 0);
        }

        void onButtonUp(const Button button) override {
            buttons = static_cast<uint8_t>(buttons & ~hidButtonBit(button));
            send(0, 0, 0);
        }

        void onMovement(const int8_t dx, const int8_t dy) override {
            if (dx == 0 && dy == 0) {
                return;
            }
            send(dx, dy, 0);
        }

        /**
         * HID wheel sign mapping -- REASONED HYPOTHESIS, NOT YET VERIFIED (#1181, 2026-11-02).
         * HID has NOT yet been tested against a real paired host; pending Edgar's test.
         *
         * Incoming ticks share calibratedYaw's sign (+ = physical +Z rotation; see
         * MouseEventSink::onScroll). Edgar's confirmed chevron spec maps +Z -> scroll DOWN
         * (FeedbackState::snapshot/onScroll). HID wheel convention is + = scroll UP, so the
         * wheel byte is the NEGATED tick count, making the real wheel agree with the chevron.
         * Done here, in the HID transport only, because GestureDetector's angle/ticks also feed
         * the already-confirmed LED chevron, which must not change. (OS "natural scrolling"
         * settings can still invert this on the host.) If Edgar's test shows the wheel opposite
         * to the chevron, remove this negation.
         */
        void onScroll(const int8_t ticks) override {
            if (ticks == 0) {
                return;
            }
            send(0, 0, hidWheelFromTicks(ticks));
        }

        /** Yaw-signed ticks -> HID wheel value (negated; -128 saturates to +127, no overflow). */
        static int8_t hidWheelFromTicks(const int8_t ticks) {
            return ticks == INT8_MIN ? static_cast<int8_t>(INT8_MAX) : static_cast<int8_t>(-ticks);
        }

        [[nodiscard]] uint8_t getButtonMask() const { return buttons; }

        /** Forget held buttons (e.g. after a disconnect) without transmitting. */
        void resetButtons() { buttons = 0; }

    private:
        void send(const int8_t dx, const int8_t dy, const int8_t wheel) const {
            if (transmit) {
                transmit(makeMouseHidReport(buttons, dx, dy, wheel));
            }
        }

        Transmit transmit;
        uint8_t buttons{0};
    };
} // namespace Hilton
