#pragma once
#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <ChainOfResponsibility.hpp>
#include <HighPassFilter.hpp>
#include <MouseButtonStateMachine.hpp>
#include <memory>
#include <string>
#include <Qmi8658C.hpp>
#include <QueuedNotificationService.hpp>

namespace Hilton {
enum MouseButtonMask : EventBits_t {
    None = 0,
    LeftButton = BIT0,
    RightButton = BIT1,
    MiddleButton = BIT2,
    WheelButton = BIT3
};
/** Mouse events: The series of detected mouse events.
 * Each event is a combination of the following:
 * - Mouse button(s) pressed/released
 * - Mouse button(s) clicked/double-clicked/triple-clicked
 * - Mouse movement
 * - Mouse wheel movement
 *
 * EXAMPLE:
 * buttons = LeftButton | MiddleButton
 * event = ButtonDown | ButtonDrag
 * deltaX = 10
 * deltaY = -20
 * wheelX = 0
 * wheelY = 0
 *
 * EXAMPLE:
 * buttons = LeftButton | MiddleButton
 * event = ButtonUp | ButtonDrag
 * deltaX = 0
 * deltaY = 0
 * wheelX = 0
 */
struct MouseEvent
{
    EventBits_t buttons;
    int8_t deltaX; // -128 to 127
    int8_t deltaY; // -128 to 127
    int8_t wheel;  // -128 to 127
};

class MouseEventDetector
{
public:
    using TrackingContext = struct
    {
        MouseButtonStateMachine leftButton{"L"};
        MouseButtonStateMachine rightButton{"R"};
        MouseButtonStateMachine middleButton{"M"};
        bool isWheelScrolling{false};
        HighPassFilter deltaXFilter{.001f};
        HighPassFilter deltaYFilter{.001f};
        HighPassFilter deltaZFilter{.001f};
        MouseEvent event{};
    };

    ~MouseEventDetector();

    MouseEventDetector(
        const std::shared_ptr<Qmi8658C> &qmi8658C, gpio_num_t int1Pin, gpio_num_t int2Pin);
    void run();

    QueuedNotificationService<MouseButtonStateMachine::State, MouseEvent, std::any>
        eventCallbackService;

    static constexpr auto TAG{"MOUSE_EVENT_DETECTOR"};

    std::shared_ptr<Qmi8658C> qmi8658c;
    gpio_num_t int1Pin;
    gpio_num_t int2Pin;

    ChainOfResponsibility<Qmi8658C::DataPoint, TrackingContext> chainOfResponsibility;
    EventGroupHandle_t dataEventGroup;
};
} // namespace Hilton