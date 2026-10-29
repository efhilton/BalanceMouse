#pragma once

#include <cstdint>

namespace Hilton::Matrix {
    /**
     * Logical <-> physical mapping for the Waveshare ESP32-S3-Matrix 8x8 WS2812B array.
     *
     * What the repo/board docs establish:
     *  - The existing neopixels component has NO naming/ordering convention:
     *    every LightPixel name is empty and getPixelByName() is unused, so the
     *    only addressing is the raw chain index 0..63 (getPixelById).
     *  - docs/ESP32-S3-Matrix-Sch.pdf: LED_DIN (GPIO14) -> U8 -> U7 ... -> U1 ->
     *    DOUT_1 -> U16 -> ... -> U9 -> DOUT_2 -> U24 ... i.e. the chain is 8 rows
     *    of 8, and every row is entered from the SAME side. That is a
     *    PROGRESSIVE (row-major) layout, NOT serpentine/zig-zag.
     *  - Waveshare-board Arduino examples configure Adafruit_NeoMatrix as
     *    NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE
     *    (chain index 0 = top-left), with NEO_RGB colour order.
     *
     * Logical coordinates used everywhere in led_feedback:
     *   x = column 0..7, left -> right; y = row 0..7, top -> bottom,
     *   looking at the LED face with the USB-C connector at the TOP.
     *   chainIndex = y * 8 + x
     *
     * ASSUMPTION (not verifiable without hardware): which physical corner is
     * index 0 and how the IMU axes line up with the LED grid. If the ball moves
     * the wrong way, flip MIRROR_X / MIRROR_Y here -- nothing else changes.
     */
    static constexpr int WIDTH{8};
    static constexpr int HEIGHT{8};
    static constexpr int PIXEL_COUNT{WIDTH * HEIGHT};

    static constexpr bool MIRROR_X{false};
    static constexpr bool MIRROR_Y{false};

    constexpr bool inBounds(const int x, const int y) {
        return x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT;
    }

    /** Logical (x, y) -> index in the LED chain (PixelsController::getPixelById). -1 if out of bounds. */
    constexpr int chainIndex(const int x, const int y) {
        if (!inBounds(x, y)) {
            return -1;
        }
        const int px = MIRROR_X ? (WIDTH - 1 - x) : x;
        const int py = MIRROR_Y ? (HEIGHT - 1 - y) : y;
        return py * WIDTH + px;
    }

    /** Logical row-major index used by Frame storage (independent of the mirror flags). */
    constexpr int logicalIndex(const int x, const int y) {
        return inBounds(x, y) ? y * WIDTH + x : -1;
    }
} // namespace Hilton::Matrix
