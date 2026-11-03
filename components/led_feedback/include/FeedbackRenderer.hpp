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

        /**
         * |scroll report angle| (0..127), 0 = not scrolling. Drives chevron animation speed:
         * slow near engagement, faster toward full deflection (#1181, Edgar: "nothing from
         * 0-10 [deg]. Past 10 degrees, scroll-down chevrons start animating slowly, speeding
         * up as the angle increases, up to a max angle"). See FeedbackRenderer::chevronStepMs.
         */
        uint8_t scrollMagnitude{0};

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
     *        scrolling ? animated chevron "treadmill" column : 1x1 tilt ball
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
        /**
         * The chevrons advance one row every `chevronStepMs(magnitude)` ms, interpolated
         * between these two (#1181, Edgar: speed ramps with deflection, not a fixed rate).
         * CHEVRON_STEP_MS is also the slowest rate, at/near engagement (magnitude ~ 0..16),
         * kept at the original fixed value so existing full-deflection-unaware callers (and
         * the original behaviour at low magnitude) are unchanged.
         */
        static constexpr uint32_t CHEVRON_STEP_MS{120};
        static constexpr uint32_t CHEVRON_STEP_MS_FAST{30};
        static constexpr int CHEVRON_PERIOD_ROWS{4};
        /**
         * Tilt angle (degrees) that should push the ball all the way to the matrix edge
         * (#1181, Edgar: shrink the ball to 1x1 and scale it to use the full travel range at
         * this angle, "more dramatic" movement). Documentation/traceability constant, same
         * pattern as GestureDetector::SCROLL_ENGAGE_DEG -- the actual enforcement is the
         * derived BALL_FULL_SCALE_TILT below; guarded by
         * test_ball_full_scale_tilt_matches_its_degree_spec.
         */
        static constexpr float BALL_FULL_SCALE_TILT_DEG{20.0f};
        /**
         * Tilt (m/s^2) equivalent of BALL_FULL_SCALE_TILT_DEG. Derived the same way as the
         * axis-swap/scroll work (#1181): at rest the accelerometer reads the gravity reaction
         * force g; tilting by angle `a` about the perpendicular axis gives a lateral component
         * of g*sin(a) (see FeedbackState::updateMotion's derivation comment). 9.81 * sin(20 deg)
         * = 9.81 * 0.34202... = 3.3552...  Recompute this if BALL_FULL_SCALE_TILT_DEG changes.
         */
        static constexpr float BALL_FULL_SCALE_TILT{3.3552f};

        // ---- Intensities (percent; all <= Lights::MAX_INTENSITY_PERCENT) ----
        // Halved from the original 2/30/40/20/40/30 set when the 25% ceiling landed (#1181,
        // Edgar's real-hardware heat finding) -- same relative design, proportionally dimmer.
        // Halved a SECOND time on 2026-11-02 (#1181, Edgar's second real-hardware heat complaint,
        // ceiling 25% -> 12%): 1/15/20/10/20/15 -> 0/7/10/5/10/7, each rounded DOWN (heat-safety
        // direction). PULSE_MIN 0 just means the idle breathing pulse bottoms out at off.
        static constexpr uint8_t PULSE_MIN_INTENSITY{0};
        static constexpr uint8_t PULSE_MAX_INTENSITY{7};
        static constexpr uint8_t FLASH_INTENSITY{10};
        static constexpr uint8_t BALL_INTENSITY{5};
        static constexpr uint8_t BUTTON_INTENSITY{10};
        static constexpr uint8_t CHEVRON_INTENSITY{7};

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

        /**
         * Ball's single pixel cell for a calibrated tilt, scaled so |tilt| == BALL_FULL_SCALE_TILT
         * reaches the matrix edge (#1181, Edgar: 1x1 ball, full travel range at 20 degrees).
         * Centred (zero tilt) = (3, 3) or (4, 4) depending on rounding -- an 8-wide grid has no
         * single exact centre pixel; std::lround's round-half-away-from-zero puts it at (4, 4).
         */
        static void ballPosition(float tiltX, float tiltY, int &outX, int &outY);

        /** Intensity of the pairing pulse at `nowMs` (smooth raised-cosine between MIN and MAX). */
        [[nodiscard]] static uint8_t pulseIntensity(uint32_t nowMs);

        /** True while a button pressed at `pressedAtMs` should currently be lit. */
        [[nodiscard]] static bool buttonIndicatorLit(uint32_t pressedAtMs, uint32_t nowMs);

        [[nodiscard]] static bool isDrag(uint32_t pressedAtMs, uint32_t nowMs);

        /**
         * Chevron row-advance period for a given |scroll angle| (0..127 report units):
         * CHEVRON_STEP_MS at magnitude 0, linearly down to CHEVRON_STEP_MS_FAST at magnitude
         * 127 (always the saturated report range regardless of GestureDetector's
         * MAX_SCROLL_ANGLE_DEG, so this needs no tuning when that changes). Public and pure so
         * it is directly testable without going through a full render().
         */
        [[nodiscard]] static uint32_t chevronStepMs(uint8_t magnitude);

    private:
        static void renderFlash(Frame &f, const RgbPixel &color, uint32_t elapsedMs);

        static void renderPulse(Frame &f, uint32_t nowMs);

        static void renderBall(Frame &f, const FeedbackSnapshot &s);

        static void renderChevrons(Frame &f, int direction, uint32_t stepMs, uint32_t nowMs);

        static void renderButtons(Frame &f, const FeedbackSnapshot &s, uint32_t nowMs);
    };
} // namespace Hilton
