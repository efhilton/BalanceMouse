#include "Qmi8658C.hpp"
using namespace Hilton;

Qmi8658C::Qmi8658C(const i2c_port_t i2CPort) : i2cPort(i2CPort)
{
}

void Qmi8658C::initialize()
{
    logCallbackService.notify("Initialized Qmi8658C sensor");
}

void Qmi8658C::onTerminate()
{
    logCallbackService.notify("Terminated Qmi8658C sensor");
}

void Qmi8658C::sample()
{
    dataCallbackService.notify(DataPoint{0, 0, 0});
}