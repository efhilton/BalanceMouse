#pragma once
#include "hal/i2c_types.h"
#include <cstdint>
#include <SimpleNotificationService.hpp>
#include <string>

namespace Hilton
{
class Qmi8658C
{
    i2c_port_t i2cPort;
public:
    using DataPoint = struct
    {
        uint8_t x;
        uint8_t y;
        uint8_t z;
    };

    explicit Qmi8658C(i2c_port_t i2CPort);
    void initialize();
    void onTerminate();
    void sample();
    SimpleNotificationService<DataPoint> dataCallbackService{};
    SimpleNotificationService<std::string> logCallbackService{};
};
}