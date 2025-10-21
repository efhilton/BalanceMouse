#pragma once
#include <map>
#include <NotFoundException.hpp>
#include <string>
#include <soc/gpio_num.h>

namespace Hilton::Lights::Pixel
{
    enum class ColorOrder
    {
        RGB = 0,
        RBG = 1,
        GRB = 2,
        GBR = 3,
        BRG = 4,
        BGR = 5,
        RGBW = 6,
        RBGW = 7,
        GRBW = 8,
        GBRW = 9,
        BRGW = 10,
        BGRW = 11,
        RESERVED_00 = 12,
        RESERVED_01 = 13,
        RESERVED_02 = 14,
        RESERVED_03 = 15
    };

    inline std::map<std::string, ColorOrder> createColorOrderMap()
    {
        return {
            {"RGB", ColorOrder::RGB},
            {"RBG", ColorOrder::RBG},
            {"GRB", ColorOrder::GRB},
            {"GBR", ColorOrder::GBR},
            {"BRG", ColorOrder::BRG},
            {"BGR", ColorOrder::BGR},
            {"RGBW", ColorOrder::RGBW},
            {"RBGW", ColorOrder::RBGW},
            {"GRBW", ColorOrder::GRBW},
            {"GBRW", ColorOrder::GBRW},
            {"BRGW", ColorOrder::BRGW},
            {"BGRW", ColorOrder::BGRW}
        };
    }

    inline std::string getColorOrderName(const ColorOrder& order)
    {
        const auto colorOrderMap = createColorOrderMap();
        for (const auto& [fst, snd] : colorOrderMap)
        {
            if (snd == order)
            {
                return fst;
            }
        }
        return "UNKNOWN";
    }

    inline ColorOrder getColorOrderFromName(const std::string& name)
    {
        if (name.empty())
        {
            throw NotFoundException("Empty color order name");
        }
        const auto colorOrderMap = createColorOrderMap();
        auto it = colorOrderMap.find(name);
        if (it != colorOrderMap.end())
        {
            return it->second;
        }
        throw NotFoundException("Unrecognized color order");
    }

    inline size_t calculateNumColors(const ColorOrder colorOrder)
    {
        const std::string name = getColorOrderName(colorOrder);
        return name.length();
    }
} // namespace Pixel

namespace Hilton::Lights::CaseLight
{
    static constexpr gpio_num_t DATA_PIN{GPIO_NUM_12};

    enum class Group { TEXT, LEFT, RIGHT, UP, DOWN, CAMERA_LIGHT };

    inline std::map<std::string, Group> createGroupMap()
    {
        return {
            {"text", Group::TEXT},
            {"arrowLeft", Group::LEFT},
            {"arrowRight", Group::RIGHT},
            {"arrowUp", Group::UP},
            {"arrowDown", Group::DOWN},
            {"cameraLight", Group::CAMERA_LIGHT}
        };
    }

    inline std::string getGroupName(const Group& group)
    {
        const auto groupMap = createGroupMap();
        for (const auto& [fst, snd] : groupMap)
        {
            if (snd == group)
            {
                return fst;
            }
        }
        return "UNKNOWN";
    }

    inline Group getGroupFromName(const std::string& name)
    {
        if (name.empty())
        {
            throw NotFoundException("Empty group name");
        }
        const auto groupMap = createGroupMap();
        auto it = groupMap.find(name);
        if (it != groupMap.end())
        {
            return it->second;
        }
        throw NotFoundException("Invalid group name");
    }

    namespace Effect
    {
        static constexpr auto ALL_OFF{"allOff"};
        static constexpr auto MANUAL{"manual"};
        static constexpr auto POLICE{"police"};
        static constexpr auto SNAKE{"snake"};
    }
}

namespace Hilton::Lights::Colors
{
    static constexpr auto HEX_ALL_OFF{"0x00000000"};
    static constexpr auto HEX_BLUE{"0x0000FF00"};
    static constexpr auto HEX_GREEN{"0x00FF0000"};
    static constexpr auto HEX_RED{"0xFF000000"};
    static constexpr auto HEX_WHITE{"0x000000FF"};
    static constexpr auto HEX_PURPLE{"0xFF00FF00"}; // Red + Blue
    static constexpr auto HEX_CYAN{"0x00FFFF00"}; // Green + Blue
    static constexpr auto HEX_PINK{"0xFF0080FF"}; // Red + half Blue + White
    static constexpr auto HEX_YELLOW{"0xFFFF0000"}; // Red + Green
    static constexpr auto HEX_AMBER{"0xFF800000"}; // Intense Red + half Green
    static constexpr auto HEX_ORANGE{"0xFFA50000"}; // Full Red + 65% Green
    static constexpr auto HEX_AQUA{"0x00FFFF00"}; // Same as cyan
    static constexpr auto HEX_TEAL{"0x00808000"}; // Half Green + Half Blue
    static constexpr auto HEX_INDIGO{"0x4B00FF00"}; // 29% Red + Full Blue

    enum class RGBColors { RED, GREEN, BLUE, WHITE, PURPLE, CYAN, PINK, YELLOW, AMBER, ORANGE, AQUA, TEAL, INDIGO };

    inline std::string getColorHex(const RGBColors& color)
    {
        const std::map<const RGBColors, const std::string> hexColors{
            {RGBColors::AMBER, HEX_AMBER},
            {RGBColors::AQUA, HEX_AQUA},
            {RGBColors::BLUE, HEX_BLUE},
            {RGBColors::CYAN, HEX_CYAN},
            {RGBColors::GREEN, HEX_GREEN},
            {RGBColors::INDIGO, HEX_INDIGO},
            {RGBColors::ORANGE, HEX_ORANGE},
            {RGBColors::PINK, HEX_PINK},
            {RGBColors::PURPLE, HEX_PURPLE},
            {RGBColors::RED, HEX_RED},
            {RGBColors::TEAL, HEX_TEAL},
            {RGBColors::WHITE, HEX_WHITE},
            {RGBColors::YELLOW, HEX_YELLOW}
        };
        return hexColors.at(color);
    }
} // namespace Colors
