#pragma once

#include <driver/rmt_tx.h>
#include <vector>
#include "led_strip_encoder.h"
#include <LightConstants.hpp>

namespace Hilton
{
    class PixelsController final
    {
    public:
        // Timings for the SK6812RGBW
        static constexpr led_strip_encoder_config_t SK6812{
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
            .T0H = 0.3,
            // 300 ns
            .T0L = 0.9,
            // 900 ns
            .T1H = 0.6,
            // 600 ns
            .T1L = 0.6,
            // 600 ns
            .TREST = 80,
            // 80,000 ns
        };

        static constexpr led_strip_encoder_config_t SK6812_TEST_STRIP{
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
            .T0H = 0.3,
            // 300 ns
            .T0L = 0.9,
            // 900 ns
            .T1H = 0.6,
            // 600 ns
            .T1L = 0.6,
            // 600 ns
            .TREST = 80,
            // 80,000 ns
        };

        static constexpr led_strip_encoder_config_t SK6812_STROBES{
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
            .T0H = 0.3,
            // 300 ns
            .T0L = 0.9,
            // 900 ns
            .T1H = 0.6,
            // 600 ns
            .T1L = 0.6,
            // 600 ns
            .TREST = 80,
            // 80,000 ns
        };

        // Timings for the WS2811 (https://cdn-shop.adafruit.com/datasheets/WS2811.pdf)
        static constexpr led_strip_encoder_config_t WS2811{
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
            .T0H = 0.5,
            // 500 ns
            .T0L = 2.0,
            // 2,000 ns
            .T1H = 1.2,
            // 1,200 ns
            .T1L = 1.3,
            // 1,300 ns
            .TREST = 50,
            // 50,000 ns
        };

        static constexpr led_strip_encoder_config_t IN_PI55QATPRPGPBPW_40{
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
            .T0H = 0.3,
            // us
            .T0L = 0.8,
            // us
            .T1H = 0.75,
            // us
            .T1L = 0.2,
            // us
            .TREST = 90,
            // us
        };

        using PixelsConfig = struct
        {
            gpio_num_t controlPin;
            int totalNumberOfPixels;
            led_strip_encoder_config_t targetDevice;
            Lights::Pixel::ColorOrder colorOrderPerPixel;
        };

        explicit PixelsController(const PixelsConfig& config);

        ~PixelsController();

        void write();

        [[nodiscard]] int getPixelIdFromName(const std::string& name) const;

        [[nodiscard]] LightPixel& getPixelById(int index);

        [[nodiscard]] LightPixel& getPixelByName(const std::string& name);

        void turnOffAll();

        esp_err_t initialize() ;

        esp_err_t terminate() ;

        [[nodiscard]] uint16_t getNumberOfPixels() const;

    private:
        gpio_num_t controlPin;
        uint16_t numberOfPixels;
        led_strip_encoder_config_t encoderConfig;
        Lights::Pixel::ColorOrder colorOrder;
        size_t numColorsPerPixel;

        rmt_channel_handle_t led_channel;
        rmt_encoder_handle_t led_encoder;
        std::vector<LightPixel> pixels;
        std::vector<uint8_t> led_strip_pixels;

        void printDebuggingInfo() const;
    };
}
