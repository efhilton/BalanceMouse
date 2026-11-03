#pragma once

#include <cstdint>

namespace Hilton::Lights {
    /**
     * Hard ceiling on per-pixel intensity (percent of full drive).
     *
     * The ESP32-S3-Matrix's 8x8 WS2812B array heats up quickly (Waveshare warns
     * that high brightness causes a rapid temperature rise). The ticket's original
     * AC set this at 50%, which every mode already stayed under (historical max
     * observed across 446,816 rendered frames was 40%) -- but Edgar's real-hardware
     * pass (#1181) still found the unit running warmer than it should at that level.
     * Lowered to 25% (half) in response; every per-mode intensity in FeedbackRenderer
     * was scaled down proportionally so the relative visual design (flash brighter
     * than ball, etc.) is unchanged, just dimmer/cooler overall. LightPixel clamps
     * in its constructor and in setIntensity(), so this holds for every caller, not
     * just the LED feedback renderer.
     *
     * 2026-11-02 (#1181): halved again in response to a second real-hardware heat complaint
     * (Edgar: "Please half the light intensity. It still gets pretty hot."). 25% -> 12%,
     * rounded DOWN (12.5 -> 12) since the concern is heat -- err cooler, not brighter. The
     * per-mode intensities in FeedbackRenderer were halved again the same way.
     */
    static constexpr uint8_t MAX_INTENSITY_PERCENT{12};

    constexpr uint8_t clampIntensity(const uint8_t intensity) {
        return intensity > MAX_INTENSITY_PERCENT ? MAX_INTENSITY_PERCENT : intensity;
    }
} // namespace Hilton::Lights
