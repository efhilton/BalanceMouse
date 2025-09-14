#include <MouseEventDetector.hpp>
#include <TinyUsbHID.hpp>

namespace Hilton {

TinyUsbHID::TinyUsbHID()
{
    // Initialize member variables, BT stack, etc.
    onData = [this](const MouseEvent &event){
        sendHidReport(event.buttons, event.deltaX, event.deltaY, event.wheel);
    };
}

TinyUsbHID::~TinyUsbHID()
{
    end();
}

bool TinyUsbHID::begin()
{
    // Start Bluetooth HID profile for a mouse
    // [platform-specific implementation]
    return true; // Return true if successful
}

void TinyUsbHID::end()
{
    // Cleanup, disconnect from host if necessary
    // [platform-specific implementation]
}

void TinyUsbHID::sendHidReport(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheelX)
{
    // Construct standard USB HID mouse report:
    // [Buttons][X][Y][Wheel]
    uint8_t report[5] = {buttons, uint8_t(dx), uint8_t(dy), uint8_t(wheelX)};

    // Transmit HID report via Bluetooth
    // -- Add your Bluetooth HID stack function here --
    // Example: btSendHIDReport(report, sizeof(report));
}

} // namespace Hilton