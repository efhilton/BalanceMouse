#pragma once
#include "hal/i2c_types.h"
#include <cstdint>

class QMI8658C {
public:
    QMI8658C(i2c_port_t i2_c_port);QMI8658C(i2c_port_t i2_c_port, void(* on_data_callback)(const uint8_t* data, const size_t len));
    void initialize();
  void onTerminate();
  void sample();
};


