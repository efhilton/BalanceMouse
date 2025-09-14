#pragma once

#include <MouseEventDetector.hpp>
#include <cstdint>

namespace Hilton {

class TinyUsbHID
{
public:
    TinyUsbHID();
    ~TinyUsbHID();

    bool begin();
    void end();
    std::function<void(const MouseEvent &data)> onData;

private:
     void sendHidReport(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheelX);
};

} // namespace Hilton