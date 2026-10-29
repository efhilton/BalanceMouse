#pragma once

#include <ConnectionState.hpp>
#include <FeedbackRenderer.hpp>
#include <MouseEventSink.hpp>
#include <cstdint>
#include <functional>
#include <mutex>

namespace Hilton {
    /**
     * Thread-safe accumulator of "what should the LEDs show". It is fed from:
     *   - the gesture event stream (it is a MouseEventSink, registered on the same
     *     fan-out as the BLE HID transport) -> button down/up + press time, scroll ticks
     *   - updateMotion() from the IMU task -> calibrated tilt for the ball, scroll state
     *   - updateConnection() from the LED task -> BLE phase + pairing outcome
     * and produces a FeedbackSnapshot for the pure FeedbackRenderer.
     */
    class FeedbackState final : public MouseEventSink {
    public:
        using Clock = std::function<uint32_t()>;

        /** A scroll tick keeps the chevrons running at least this long (covers slow tick rates). */
        static constexpr uint32_t SCROLL_HOLD_MS{250};

        explicit FeedbackState(Clock clock) : clock(std::move(clock)) {
        }

        void onButtonDown(Button button) override;

        void onButtonUp(Button button) override;

        void onMovement(int8_t dx, int8_t dy) override;

        void onScroll(int8_t ticks) override;

        /** Calibrated tilt and current scroll deflection (0 when the wheel is inactive). */
        void updateMotion(float tiltX, float tiltY, int8_t scrollAngle);

        /**
         * Latest transport state. A pairing status whose `sequence` differs from the
         * last one seen is treated as a NEW outcome and timestamped now.
         */
        void updateConnection(ConnectionPhase phase, const PairingStatus &pairing);

        [[nodiscard]] FeedbackSnapshot snapshot() const;

    private:
        static int indexOf(Button button);

        Clock clock;
        mutable std::mutex mutex;
        FeedbackSnapshot state{};
        int8_t scrollAngle{0};
        int8_t lastTickDirection{0};
        uint32_t lastTickMs{0};
        bool hasTick{false};
        uint32_t lastPairingSequence{0};
    };
} // namespace Hilton
