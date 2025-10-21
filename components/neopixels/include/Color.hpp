#pragma once

#include <ArgumentException.hpp>
#include <iomanip>
#include <memory>
#include <regex>
#include <sstream>

namespace Hilton {
    class Color {
    public:
        static constexpr auto COLOR_REGEX{R"(^((0x)|(#))([0-9A-F]{6}|[0-9A-F]{8})$)"};

        Color() = default;

        ~Color() = default;

        Color(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w)
            : red(r),
              green(g),
              blue(b),
              white(w) {
        }

        Color(const Color &c)
            : red(c.getRed()),
              green(c.getGreen()),
              blue(c.getBlue()),
              white(c.getWhite()) {
        }

        explicit Color(const std::string &color) {
            setColor(color);
        }

        Color &operator=(const Color &other) {
            if (this != &other) {
                red = other.getRed();
                green = other.getGreen();
                blue = other.getBlue();
                white = other.getWhite();
            }
            return *this;
        }

        /**
         * @brief Set the color of the object using another Color object.
         *
         * This method sets the color of the object using the red, green, blue, and white components of another Color object.
         *
         * @param c The Color object containing the desired color.
         *
         * @see Color
         *
         * @param c A shared pointer to a Color object representing the desired color.
         *
         * @code
         * std::shared_ptr<Color> color = std::make_shared<Color>(red, green, blue, white);
         * setColor(color);
         * @endcode
         */
        void setColor(const std::shared_ptr<Color> &c) {
            setColor(c->getRed(), c->getGreen(), c->getBlue(), c->getWhite());
        }

        /**
         * @brief Set the color of the object using another Color object.
         *
         * This method sets the color of the object using the red, green, blue, and white components of another Color object.
         *
         * @param c The Color object containing the desired color.
         *
         * @see Color
         */
        void setColor(const Color &c) {
            setColor(c.getRed(), c.getGreen(), c.getBlue(), c.getWhite());
        }

        /**
         * @brief Set the color of the object.
         *
         * This method takes four parameters representing the red (r), green (g), blue (b), and white (w) components of the color and sets the color of the object accordingly.
         *
         * @param r The red component of the color (0-255).
         * @param g The green component of the color (0-255).
         * @param b The blue component of the color (0-255).
         * @param w The white component of the color (0-255).
         *
         * @throws N/A
         */
        void setColor(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w) {
            setRed(r);
            setGreen(g);
            setBlue(b);
            setWhite(w);
        }

        /**
         * @brief Set the color of the object.
         *
         * This method takes a color string in the format "0xRRGGBBWW" or "0xRRGGBB" and sets the color of the object accordingly.
         * The color string should start with "0x" followed by two hexadecimal digits representing the red component, two hexadecimal digits representing the green component,
         * two hexadecimal digits representing the blue component, and optionally, two hexadecimal digits representing the white component.
         *
         * @param colorString The color string in the format "0xRRGGBBWW" or "0xRRGGBB" representing the color.
         *
         * @throws ArgumentException if the color string is invalid.
         */
        void setColor(const std::string &colorString) {
            validateColorString(colorString);
            const std::string s = colorString.substr(colorString[0] == '#' ? 1 : 2); // Skip '0x' or '#'
            uint32_t color;
            std::stringstream ss;
            ss << std::hex << s;
            ss >> color;
            uint8_t r{0};
            uint8_t g{0};
            uint8_t b{0};
            uint8_t w{0};

            // If an alpha channel is included (length = 8 for hex, length = 9 if starts with '#')
            if (s.length() == 8) {
                r = (color >> 24) & 0xFF;
                g = (color >> 16) & 0xFF;
                b = (color >> 8) & 0xFF;
                w = color & 0xFF;
            } else {
                // If no alpha channel (length = 6 for hex, length = 7 if starts with '#')
                r = (color >> 16) & 0xFF;
                g = (color >> 8) & 0xFF;
                b = color & 0xFF;
                w = 0;
            }

            setRed(r);
            setGreen(g);
            setBlue(b);
            setWhite(w);
        }

        /**
         * @brief Convert the color object to a string representation.
         *
         * This method converts the color object to a string representation in the format "0xRRGGBBWW", where RR represents the red component,
         * GG represents the green component, BB represents the blue component, and WW represents the white component.
         * Each component is represented by two hexadecimal digits.
         *
         * @return A string representation of the color object.
         *
         * @see Color
         */
        [[nodiscard]] std::string toString() const {
            std::stringstream ss;
            ss << std::hex << std::uppercase;
            ss << std::setw(2) << std::setfill('0') << static_cast<int>(red);
            ss << std::setw(2) << std::setfill('0') << static_cast<int>(green);
            ss << std::setw(2) << std::setfill('0') << static_cast<int>(blue);
            ss << std::setw(2) << std::setfill('0') << static_cast<int>(white);
            return "#" + ss.str();
        }

        /**
         * @brief Validate a color string.
         *
         * This method validates a color string to ensure that it is in the correct format. The color string should start with "0x" or "#" followed by either 6 or 8 hexadecimal digits representing the red, green, blue, and optional white components of the color.
         *
         * @param colorString The color string to validate.
         *
         * @throws ArgumentException if the color string is invalid.
         *
         * @see Color, ArgumentException
         */
        static void validateColorString(const std::string &colorString) {
            const std::regex regex{COLOR_REGEX};
            if (!std::regex_match(colorString, regex)) {
                throw ArgumentException("Invalid color string. The format should be 0xRRGGBBWW, 0xRRGGBB, #RRGGBBWW, or #RRGGBB.");
            }
        }

        [[nodiscard]] uint8_t getRed() const {
            return red;
        }

        [[maybe_unused]] Color &setRed(const uint8_t newRed) {
            red = newRed;
            return *this;
        }

        [[nodiscard]] uint8_t getGreen() const {
            return green;
        }

        Color &setGreen(const uint8_t newGreen) {
            green = newGreen;
            return *this;
        }

        [[nodiscard]] uint8_t getBlue() const {
            return blue;
        }

        Color &setBlue(const uint8_t newBlue) {
            blue = newBlue;
            return *this;
        }

        [[nodiscard]] uint8_t getWhite() const {
            return white;
        }

        Color &setWhite(const uint8_t newWhite) {
            white = newWhite;
            return *this;
        }

    private:
        uint8_t red{};
        uint8_t green{};
        uint8_t blue{};
        uint8_t white{};
    };
}
