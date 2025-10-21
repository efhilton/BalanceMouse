#pragma once

#include <algorithm>
#include <Color.hpp>
#include <string>

namespace Hilton {
    class LightPixel : public Color {
    public:
        LightPixel() : LightPixel(Color{}, 1) {
        }

        LightPixel(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w, const uint8_t intensity) : LightPixel(Color(r, g, b, w), intensity) {
        }

        explicit LightPixel(const Color &color, const uint8_t intensity) : Color(color), intensity(intensity) {
        }

        ~LightPixel()  = default;

        [[nodiscard]] uint32_t asWideWord() const {
            return getRed() << 24 | getGreen() << 16 | getBlue() << 8 | getWhite();
        }

        [[nodiscard]] const std::string &getName() const {
            return name;
        }

        LightPixel &setName(const std::string &newName) {
            name = newName;
            return *this;
        }

        [[nodiscard]] uint8_t getIntensity() const {
            return intensity;
        }

        LightPixel &setIntensity(const uint8_t intensity) {
            this->intensity = std::min(intensity, static_cast<uint8_t>(100));
            return *this;
        }

        [[nodiscard]] uint8_t getScaledRed() const {
            return scaleColorValue(getRed(), intensity);
        }

        [[nodiscard]] uint8_t getScaledGreen() const {
            return scaleColorValue(getGreen(), intensity);
        }

        [[nodiscard]] uint8_t getScaledBlue() const {
            return scaleColorValue(getBlue(), intensity);
        }

        [[nodiscard]] uint8_t getScaledWhite() const {
            return scaleColorValue(getWhite(), intensity);
        }

        /**
         * Scales the color value based on the given scale.
         *
         * @param original The original color value to scale.
         * @param scale The scale to apply to the original color value.
         * @return The scaled color value as an integer.
         */
        static int scaleColorValue(const uint8_t original, const uint8_t scale) {
            return static_cast<int>((static_cast<float>(scale) * static_cast<float>(original)) / 100.0f);
        }

    private:
        std::string name;
        uint8_t intensity;
    };
}