#include <iomanip>
#include <iostream>
#include <led_strip_encoder.h>
#include <LightPixel.hpp>
#include <NotImplementedException.hpp>
#include <PixelsController.hpp>
#include <sstream>
#include <stdexcept>
#include <freertos/FreeRTOS.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"

using namespace Hilton;

static constexpr rmt_transmit_config_t TX_CONFIG = {
    .loop_count = 0,
    // no transfer loop
    .flags{}
};

static constexpr const char* TAG{"PIXELS_CONTROLLER"};


PixelsController::PixelsController(const PixelsConfig& config)
    : controlPin(config.controlPin),
      numberOfPixels(config.totalNumberOfPixels),
      encoderConfig(config.targetDevice),
      colorOrder(config.colorOrderPerPixel),
      numColorsPerPixel(calculateNumColors(colorOrder)),
      led_channel(),
      led_encoder(),
      pixels(numberOfPixels),
      led_strip_pixels(numColorsPerPixel * numberOfPixels)
{
    ESP_LOGI(TAG,
             "Constructor finished with numColorsPerPixel=%d, numberOfPixels=%d, controlPin=%d, colorOrder=%s",
             numColorsPerPixel,
             numberOfPixels,
             controlPin,
             Lights::Pixel::getColorOrderName(colorOrder).c_str());
}

PixelsController::~PixelsController() = default;

void PixelsController::write()
{
    if (led_strip_pixels.empty() || pixels.empty())
    {
        return;
    }

    for (int i = 0; i < numberOfPixels; i++)
    {
        switch (colorOrder)
        {
        case Lights::Pixel::ColorOrder::RGB:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledBlue();
            break;
        case Lights::Pixel::ColorOrder::RGBW:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 3] = pixels[i].getScaledWhite();
            break;
        case Lights::Pixel::ColorOrder::GRBW:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 3] = pixels[i].getScaledWhite();
            break;
        case Lights::Pixel::ColorOrder::GRB:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledBlue();
            break;
        case Lights::Pixel::ColorOrder::RBG:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledGreen();
            break;
        case Lights::Pixel::ColorOrder::GBR:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledRed();
            break;
        case Lights::Pixel::ColorOrder::BRG:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledGreen();
            break;
        case Lights::Pixel::ColorOrder::BGR:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledRed();
            break;
        case Lights::Pixel::ColorOrder::GBRW:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 3] = pixels[i].getScaledWhite();
            break;
        case Lights::Pixel::ColorOrder::BRGW:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 3] = pixels[i].getScaledWhite();
            break;
        case Lights::Pixel::ColorOrder::BGRW:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 3] = pixels[i].getScaledWhite();
            break;
        case Lights::Pixel::ColorOrder::RBGW:
            led_strip_pixels[i * numColorsPerPixel + 0] = pixels[i].getScaledRed();
            led_strip_pixels[i * numColorsPerPixel + 1] = pixels[i].getScaledBlue();
            led_strip_pixels[i * numColorsPerPixel + 2] = pixels[i].getScaledGreen();
            led_strip_pixels[i * numColorsPerPixel + 3] = pixels[i].getScaledWhite();
            break;
        default:
            throw NotImplementedException("Unsupported color order");
        }
    }

    // printDebuggingInfo();
    ESP_ERROR_CHECK(rmt_transmit(led_channel, led_encoder, led_strip_pixels.data(), led_strip_pixels.size(), &TX_CONFIG));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_channel, -1));
}

static std::string formatVectorWithDivider(const std::vector<uint8_t>& vec, const size_t divisionAfterNumRegisters)
{
    std::stringstream ss{};

    ss << "| ";
    for (size_t i = 0; i < vec.size(); ++i)
    {
        // Format each element
        ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(vec[i]) << " ";

        // Add a divider after every 'divisionAfterNumRegisters' elements, but not at the end of the string
        if ((i + 1) % divisionAfterNumRegisters == 0)
        {
            ss << "| ";
        }
    }
    return ss.str();
}

void PixelsController::printDebuggingInfo() const
{
    std::cout << formatVectorWithDivider(led_strip_pixels, numColorsPerPixel) << std::endl;
}

[[nodiscard]] int PixelsController::getPixelIdFromName(const std::string& name) const
{
    for (int i = 0; i < getNumberOfPixels(); i++)
    {
        if (pixels[i].getName() == name)
        {
            return i;
        }
    }
    throw std::out_of_range("Pixel not found.");
}

[[nodiscard]] LightPixel& PixelsController::getPixelById(const int index)
{
    if (index < 0 || index > numberOfPixels - 1)
    {
        ESP_LOGW(TAG, "Index out of range: %d not in [0,%d]. Aborting.", index, numberOfPixels - 1);
        throw std::out_of_range("Requested index is out of range.");
    }
    return pixels[index];
}

[[nodiscard]] LightPixel& PixelsController::getPixelByName(const std::string& name)
{
    for (int i = 0; i < getNumberOfPixels(); i++)
    {
        if (pixels[i].getName() == name)
        {
            return pixels[i];
        }
    }
    throw std::out_of_range("Pixel not found.");
}

void PixelsController::turnOffAll()
{
    for (int i = 0; i < numberOfPixels; i++)
    {
        pixels[i].setColor(0, 0, 0, 0);
        pixels[i].setIntensity(0);
    }
    write();
}

esp_err_t PixelsController::initialize()
{
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num = controlPin;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT; // select a source clock
    tx_chan_config.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = 64; // increase the block size can make the LED less flickering
    tx_chan_config.trans_queue_depth = 4; // set the number of transactions that can be pending in the background

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_channel));

    ESP_LOGI(TAG, "Install led strip encoder");
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoderConfig, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_channel));
    turnOffAll();
    return ESP_OK;
}

esp_err_t PixelsController::terminate()
{
    turnOffAll();
    ESP_ERROR_CHECK(rmt_disable(this->led_channel));
    ESP_ERROR_CHECK(rmt_del_channel(this->led_channel));
    ESP_ERROR_CHECK(rmt_del_encoder(this->led_encoder));
    return ESP_OK;
}


[[nodiscard]] uint16_t PixelsController::getNumberOfPixels() const
{
    return numberOfPixels;
}
