#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "soc/gpio_num.h"
#include <memory>
#include <Qmi8658C.hpp>

namespace Hilton {
class MouseEventDetector
{
    static constexpr auto TAG{"MOUSE_EVENT_DETECTOR"};

    std::shared_ptr<Qmi8658C> qmi8658c;
    SimpleNotificationService<Qmi8658C::DataPoint>::OnDataCallback onDataCallback;
    gpio_num_t int1Pin;
    gpio_num_t int2Pin;
    EventGroupHandle_t eventGroup;

    static void analyze(const Qmi8658C::DataPoint &data);

public:
    enum MouseButtonMask { LeftButton = 0x01, RightButton = 0x02, MiddleButton = 0x04 };
    enum MouseEventMask {
        ButtonDown = 0x01,
        ButtonUp = 0x02,
        ButtonClick = 0x04,
        ButtonDoubleClick = 0x08,
        ButtonTripleClick = 0x10,
        ButtonDrag = 0x20
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
        uint8_t buttons;
        uint8_t event;
        int8_t deltaX; // -128 to 127
        int8_t deltaY; // -128 to 127
        int8_t wheelX; // -128 to 127
        int8_t wheelY; // -128 to 127
    };

    ~MouseEventDetector() = default;

    SimpleNotificationService<MouseEvent> eventCallbackService{};

    static void gpioIsrHandler(void *p);
    MouseEventDetector(
        const std::shared_ptr<Qmi8658C> &qmi8658C, gpio_num_t int1Pin, gpio_num_t int2Pin);
    void run();
};
} // namespace Hilton