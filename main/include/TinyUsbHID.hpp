#pragma once

#include <MouseEventDetector.hpp>
#include <cstdint>

namespace Hilton {


class TinyUsbHID
{
public:
    TinyUsbHID();
    ~TinyUsbHID();

    // Call to initialize and enable the Bluetooth mouse device
    bool begin();

    // Send a mouse event as an HID report
    void sendMouseEvent(const MouseEventDetector::MouseEvent &event);

    // (optional) Disconnect or clean up
    void end();

private:
    // Sends raw HID report over Bluetooth (implementation platform-specific)
    void sendHIDReport(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheelX);

    // Internal state, connection handle, etc (platform-specific)
    // Example: int btConnectionHandle;
};

} // namespace Hilton