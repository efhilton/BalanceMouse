#pragma once

#include <ConnectionState.hpp>
#include <LightLimits.hpp>
#include <MatrixLayout.hpp>
#include <array>
#include <cstdint>

namespace Hilton {
    /** One LED: full-scale colour plus an intensity percentage (always <= MAX_INTENSITY_PERCENT). */
    struct RgbPixel {
        uint8_t r{0};
        uint8_t g{0};
        uint8_t b{0};
        uint8_t intensity{0};

        [[nodiscard]] bool isLit() const { return intensity > 0 && (r | g | b) != 0; }

        bool operator==(const RgbPixel &o) const {
            return r == o.r && g == o.g && b == o.b && intensity == o.intensity;
        }
    };

    /** 8x8 frame in LOGICAL coordinates (see MatrixLayout.hpp). */
    class Frame {
    public:
        [[nodiscard]] const RgbPixel &at(const int x, const int y) const { return pixels[Matrix::logicalIndex(x, y)]; }

        /** Out-of-bounds writes are ignored. Intensity is clamped to the 50% ceiling. */
        void set(const int x, const int y, const RgbPixel &p) {
            const int i = Matrix::logicalIndex(x, y);
            if (i < 0) {
                return;
            }
            pixels[i] = p;
            pixels[i].intensity = Lights::clampIntensity(p.intensity);
        }

        void fill(const RgbPixel &p) {
            for (int y = 0; y < Matrix::HEIGHT; ++y) {
                for (int x = 0; x < Matrix::WIDTH; ++x) {
                    set(x, y, p);
                }
            }
        }

        [[nodiscard]] const std::array<RgbPixel, Matrix::PIXEL_COUNT> &raw() const { return pixels; }

        bool operator==(const Frame &o) const { return pixels == o.pixels; }
        bool operator!=(const Frame &o) const { return !(*this == o); }

    private:
        std::array<RgbPixel, Matrix::PIXEL_COUNT> pixels{};
    };

    /** Everything the renderer needs to know about the app, captured at one instant. */
    struct FeedbackSnapshot {
        ConnectionPhase phase{ConnectionPhase::Idle};

        /** Most recent pairing outcome and when (ms) it was observed. None = no outcome yet. */
        PairingOutcome pairingOutcome{PairingOutcome::None};
        uint32_t pairingOutcomeAtMs{0};

        /** Indexed by button: 0 = Left, 1 = Right, 2 = Middle. */
        std::array<bool, 3> buttonPressed{};
        std::array<uint32_t, 3> buttonPressedAtMs{};

        /** +1 scroll up, -1 scroll down, 0 = not scrolling. */
        int8_t scrollDirection{0};

        /** Calibrated tilt (m/s^2, baseline already subtracted). +x = right, +y = down (same as cursor). */
        float tiltX{0};
        float tiltY{0};
    };

    /**
     * PURE function from (app state, time) -> 64 pixel colours + intensities.
     * No hardware, no clocks, no globals: this is what host_test exercises.
     *
     * Priority (highest first):
     *   1. pairing success flash (2x green) / failure flash (3x red), if still within its window
     *   2. not Connected (Idle/Advertising/Connecting) -> slow blue breathing pulse over the whole matrix
     *   3. Connected -> mouse display:
     *        scrolling ? animated chevron "treadmill" column : 2x2 tilt ball
     *        + button overlay: Left = far-left column (green), Right = far-right column (amber),
     *          Middle = top row (cyan); solid while a click is short, blinking once held
     *          past DRAG_DWELL_MS ("drag" display state).
     */
    class FeedbackRenderer {
    public:
        enum class Mode { SuccessFlash, FailureFlash, PairingPulse, Mouse };

        // ---- Timing (ms) ----
        static constexpr uint32_t FLASH_ON_MS{150};
        static constexpr uint32_t FLASH_OFF_MS{150};
        static constexpr int SUCCESS_FLASH_COUNT{2};
        static constexpr int FAILURE_FLASH_COUNT{3};
        static constexpr uint32_t SUCCESS_FLASH_TOTAL_MS{SUCCESS_FLASH_COUNT * (FLASH_ON_MS + FLASH_OFF_MS)};
        static constexpr uint32_t FAILURE_FLASH_TOTAL_MS{FAILURE_FLASH_COUNT * (FLASH_ON_MS + FLASH_OFF_MS)};
        static constexpr uint32_t PULSE_PERIOD_MS{3000};
        /** A button still held after this long is shown as a drag (blinking). Judgment call. */
        static constexpr uint32_t DRAG_DWELL_MS{500};
        static constexpr uint32_t DRAG_BLINK_HALF_PERIOD_MS{200};
        /** The chevrons advance one row per step. */
        static constexpr uint32_t CHEVRON_STEP_MS{120};
        static constexpr int CHEVRON_PERIOD_ROWS{4};
        /** Tilt (m/s^2) that pushes the ball all the way to the edge (~30 degrees). */
        static constexpr float BALL_FULL_SCALE_TILT{4.9f};

        // ---- Intensities (percent; all <= Lights::MAX_INTENSITY_PERCENT) ----
        static constexpr uint8_t PULSE_MIN_INTENSITY{2};
        static constexpr uint8_t PULSE_MAX_INTENSITY{30};
        static constexpr uint8_t FLASH_INTENSITY{40};
        static constexpr uint8_t BALL_INTENSITY{20};
        static constexpr uint8_t BUTTON_INTENSITY{40};
        static constexpr uint8_t CHEVRON_INTENSITY{30};

        // ---- Colours (full-scale RGB; intensity applied separately) ----
        static constexpr RgbPixel PULSE_BLUE{0, 0, 255, 0};
        static constexpr RgbPixel SUCCESS_GREEN{0, 255, 0, FLASH_INTENSITY};
        static constexpr RgbPixel FAILURE_RED{255, 0, 0, FLASH_INTENSITY};
        static constexpr RgbPixel BALL_WHITE{255, 255, 255, BALL_INTENSITY};
        static constexpr RgbPixel LEFT_GREEN{0, 255, 0, BUTTON_INTENSITY};
        static constexpr RgbPixel RIGHT_AMBER{255, 128, 0, BUTTON_INTENSITY};
        static constexpr RgbPixel MIDDLE_CYAN{0, 255, 255, BUTTON_INTENSITY};
        static constexpr RgbPixel CHEVRON_PURPLE{180, 0, 255, CHEVRON_INTENSITY};

        static_assert(PULSE_MAX_INTENSITY <= Lights::MAX_INTENSITY_PERCENT);
        static_assert(FLASH_INTENSITY <= Lights::MAX_INTENSITY_PERCENT);
        static_assert(BALL_INTENSITY <= Lights::MAX_INTENSITY_PERCENT);
        static_assert(BUTTON_INTENSITY <= Lights::MAX_INTENSITY_PERCENT);
        static_assert(CHEVRON_INTENSITY <= Lights::MAX_INTENSITY_PERCENT);

        [[nodiscard]] static Mode modeFor(const FeedbackSnapshot &s, uint32_t nowMs);

        [[nodiscard]] static Frame render(const FeedbackSnapshot &s, uint32_t nowMs);

        /** Ball's top-left cell for a calibrated tilt; the ball covers (x..x+1, y..y+1). Centred = (3, 3). */
        static void ballPosition(float tiltX, float tiltY, int &outX, int &outY);

        /** Intensity of the pairing pulse at `nowMs` (smooth raised-cosine between MIN and MAX). */
        [[nodiscard]] static uint8_t pulseIntensity(uint32_t nowMs);

        /** True while a button pressed at `pressedAtMs` should currently be lit. */
        [[nodiscard]] static bool buttonIndicatorLit(uint32_t pressedAtMs, uint32_t nowMs);

        [[nodiscard]] static bool isDrag(uint32_t pressedAtMs, uint32_t nowMs);

    private:
        static void renderFlash(Frame &f, const RgbPixel &color, uint32_t elapsedMs);

        static void renderPulse(Frame &f, uint32_t nowMs);

        static void renderBall(Frame &f, const FeedbackSnapshot &s);

        static void renderChevrons(Frame &f, int direction, uint32_t nowMs);

        static void renderButtons(Frame &f, const FeedbackSnapshot &s, uint32_t nowMs);
    };
} // namespace Hilton
