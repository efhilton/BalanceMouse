#pragma once

#include <cstdint>

namespace Hilton {
    /**
     * Debounced short/long press classifier for a single push button (the BOOT
     * button). Pure logic -- the GPIO ISR / polling task lives in main
     * (BootButton.cpp) and just feeds raw levels + timestamps in here.
     *
     *  - A level change only counts once it has been stable for `debounceMs`.
     *  - LongPress fires the moment the button has been held `longPressMs`
     *    (measured from the debounced press edge), WITHOUT waiting for release.
     *  - ShortPress fires on a debounced release that happens before that.
     *  - The release that follows a LongPress produces no event.
     */
    class ButtonPressDetector {
    public:
        enum class Event { None, ShortPress, LongPress };

        static constexpr uint32_t DEFAULT_DEBOUNCE_MS{30};
        static constexpr uint32_t DEFAULT_LONG_PRESS_MS{3000};

        explicit ButtonPressDetector(const uint32_t debounceMs = DEFAULT_DEBOUNCE_MS,
                                     const uint32_t longPressMs = DEFAULT_LONG_PRESS_MS)
            : debounceMs(debounceMs), longPressMs(longPressMs) {
        }

        /** Feed the current raw level (true = pressed) at time `nowMs`. */
        Event update(const bool rawPressed, const uint32_t nowMs) {
            if (rawPressed != lastRaw) {
                lastRaw = rawPressed;
                lastRawChangeMs = nowMs;
            }

            Event event = Event::None;
            if (lastRaw != stablePressed && (nowMs - lastRawChangeMs) >= debounceMs) {
                stablePressed = lastRaw;
                if (stablePressed) {
                    pressStartMs = lastRawChangeMs;
                    longPressFired = false;
                } else if (!longPressFired) {
                    event = Event::ShortPress;
                }
            }

            // Require the RAW level to still be pressed too: a release that is only
            // waiting out its debounce window just before the 3 s mark is a short press.
            if (stablePressed && lastRaw && !longPressFired && (nowMs - pressStartMs) >= longPressMs) {
                longPressFired = true;
                event = Event::LongPress;
            }
            return event;
        }

        /** Debounced state. */
        [[nodiscard]] bool isPressed() const { return stablePressed; }

        /** True while the caller should keep polling (button held or level not yet settled). */
        [[nodiscard]] bool needsPolling() const { return stablePressed || lastRaw != stablePressed; }

    private:
        uint32_t debounceMs;
        uint32_t longPressMs;
        bool lastRaw{false};
        bool stablePressed{false};
        bool longPressFired{false};
        uint32_t lastRawChangeMs{0};
        uint32_t pressStartMs{0};
    };
} // namespace Hilton
