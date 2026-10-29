#include <FeedbackRenderer.hpp>
#include <algorithm>
#include <cmath>

namespace Hilton {
    namespace {
        constexpr float PI{3.14159265358979f};

        RgbPixel withIntensity(RgbPixel p, const uint8_t intensity) {
            p.intensity = Lights::clampIntensity(intensity);
            return p;
        }
    } // namespace

    FeedbackRenderer::Mode FeedbackRenderer::modeFor(const FeedbackSnapshot &s, const uint32_t nowMs) {
        if (s.pairingOutcome != PairingOutcome::None && nowMs >= s.pairingOutcomeAtMs) {
            const uint32_t elapsed = nowMs - s.pairingOutcomeAtMs;
            if (s.pairingOutcome == PairingOutcome::Success && elapsed < SUCCESS_FLASH_TOTAL_MS) {
                return Mode::SuccessFlash;
            }
            if (s.pairingOutcome == PairingOutcome::Failed && elapsed < FAILURE_FLASH_TOTAL_MS) {
                return Mode::FailureFlash;
            }
        }
        if (s.phase != ConnectionPhase::Connected) {
            return Mode::PairingPulse;
        }
        return Mode::Mouse;
    }

    Frame FeedbackRenderer::render(const FeedbackSnapshot &s, const uint32_t nowMs) {
        Frame f{};
        switch (modeFor(s, nowMs)) {
            case Mode::SuccessFlash:
                renderFlash(f, SUCCESS_GREEN, nowMs - s.pairingOutcomeAtMs);
                break;
            case Mode::FailureFlash:
                renderFlash(f, FAILURE_RED, nowMs - s.pairingOutcomeAtMs);
                break;
            case Mode::PairingPulse:
                renderPulse(f, nowMs);
                break;
            case Mode::Mouse:
                if (s.scrollDirection != 0) {
                    renderChevrons(f, s.scrollDirection, nowMs);
                } else {
                    renderBall(f, s);
                }
                renderButtons(f, s, nowMs);
                break;
        }
        return f;
    }

    void FeedbackRenderer::renderFlash(Frame &f, const RgbPixel &color, const uint32_t elapsedMs) {
        const bool on = (elapsedMs % (FLASH_ON_MS + FLASH_OFF_MS)) < FLASH_ON_MS;
        if (on) {
            f.fill(color);
        }
    }

    uint8_t FeedbackRenderer::pulseIntensity(const uint32_t nowMs) {
        const float phase = static_cast<float>(nowMs % PULSE_PERIOD_MS) / static_cast<float>(PULSE_PERIOD_MS);
        const float level = 0.5f * (1.0f - std::cos(2.0f * PI * phase)); // 0 -> 1 -> 0, smooth
        const float value = static_cast<float>(PULSE_MIN_INTENSITY)
                            + level * static_cast<float>(PULSE_MAX_INTENSITY - PULSE_MIN_INTENSITY);
        return Lights::clampIntensity(static_cast<uint8_t>(std::lround(value)));
    }

    void FeedbackRenderer::renderPulse(Frame &f, const uint32_t nowMs) {
        f.fill(withIntensity(PULSE_BLUE, pulseIntensity(nowMs)));
    }

    void FeedbackRenderer::ballPosition(const float tiltX, const float tiltY, int &outX, int &outY) {
        constexpr float CENTER{(Matrix::WIDTH - 2) / 2.0f}; // 3: ball covers cols/rows 3..4
        auto axis = [](const float tilt) {
            float n = std::isnan(tilt) ? 0.0f : tilt / BALL_FULL_SCALE_TILT;
            n = std::clamp(n, -1.0f, 1.0f);
            return static_cast<int>(std::lround(CENTER + n * CENTER));
        };
        outX = std::clamp(axis(tiltX), 0, Matrix::WIDTH - 2);
        outY = std::clamp(axis(tiltY), 0, Matrix::HEIGHT - 2);
    }

    void FeedbackRenderer::renderBall(Frame &f, const FeedbackSnapshot &s) {
        int bx = 0;
        int by = 0;
        ballPosition(s.tiltX, s.tiltY, bx, by);
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                f.set(bx + dx, by + dy, BALL_WHITE);
            }
        }
    }

    void FeedbackRenderer::renderChevrons(Frame &f, const int direction, const uint32_t nowMs) {
        // A centred column of 2-row chevrons, one every CHEVRON_PERIOD_ROWS rows,
        // shifted by one row every CHEVRON_STEP_MS in the scroll direction.
        //   up (+1):  ..XX..     down (-1): .X..X.
        //            .X..X.                ..XX..
        const int phase = static_cast<int>((nowMs / CHEVRON_STEP_MS) % CHEVRON_PERIOD_ROWS);
        for (int y = 0; y < Matrix::HEIGHT; ++y) {
            // Moving up: pattern row index increases as phase increases -> content moves toward y=0.
            const int k = direction > 0
                              ? (y + phase) % CHEVRON_PERIOD_ROWS
                              : ((y - phase) % CHEVRON_PERIOD_ROWS + CHEVRON_PERIOD_ROWS) % CHEVRON_PERIOD_ROWS;
            const bool apexRow = direction > 0 ? (k == 0) : (k == 1);
            const bool armRow = direction > 0 ? (k == 1) : (k == 0);
            if (apexRow) {
                f.set(3, y, CHEVRON_PURPLE);
                f.set(4, y, CHEVRON_PURPLE);
            } else if (armRow) {
                f.set(2, y, CHEVRON_PURPLE);
                f.set(5, y, CHEVRON_PURPLE);
            }
        }
    }

    bool FeedbackRenderer::isDrag(const uint32_t pressedAtMs, const uint32_t nowMs) {
        return nowMs >= pressedAtMs && (nowMs - pressedAtMs) >= DRAG_DWELL_MS;
    }

    bool FeedbackRenderer::buttonIndicatorLit(const uint32_t pressedAtMs, const uint32_t nowMs) {
        if (!isDrag(pressedAtMs, nowMs)) {
            return true; // click: solid
        }
        const uint32_t sinceDrag = nowMs - pressedAtMs - DRAG_DWELL_MS;
        return ((sinceDrag / DRAG_BLINK_HALF_PERIOD_MS) % 2) == 0; // drag: blinking, starts "on"
    }

    void FeedbackRenderer::renderButtons(Frame &f, const FeedbackSnapshot &s, const uint32_t nowMs) {
        if (s.buttonPressed[0] && buttonIndicatorLit(s.buttonPressedAtMs[0], nowMs)) {
            for (int y = 0; y < Matrix::HEIGHT; ++y) f.set(0, y, LEFT_GREEN);
        }
        if (s.buttonPressed[1] && buttonIndicatorLit(s.buttonPressedAtMs[1], nowMs)) {
            for (int y = 0; y < Matrix::HEIGHT; ++y) f.set(Matrix::WIDTH - 1, y, RIGHT_AMBER);
        }
        if (s.buttonPressed[2] && buttonIndicatorLit(s.buttonPressedAtMs[2], nowMs)) {
            for (int x = 0; x < Matrix::WIDTH; ++x) f.set(x, 0, MIDDLE_CYAN);
        }
    }
} // namespace Hilton
