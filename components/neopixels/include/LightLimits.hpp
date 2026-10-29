#pragma once

#include <cstdint>

namespace Hilton::Lights {
    /**
     * Hard ceiling on per-pixel intensity (percent of full drive).
     *
     * The ESP32-S3-Matrix's 8x8 WS2812B array heats up quickly (Waveshare warns
     * that high brightness causes a rapid temperature rise), so NOTHING may
     * drive a pixel above 50%. LightPixel clamps in its constructor and in
     * setIntensity(), so this holds for every caller, not just the LED
     * feedback renderer.
     */
    static constexpr uint8_t MAX_INTENSITY_PERCENT{50};

    constexpr uint8_t clampIntensity(const uint8_t intensity) {
        return intensity > MAX_INTENSITY_PERCENT ? MAX_INTENSITY_PERCENT : intensity;
    }
} // namespace Hilton::Lights
