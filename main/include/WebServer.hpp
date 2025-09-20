#pragma once

#include <functional>
#include <memory>
#include <string>

namespace Hilton {

class BleHidMouse;

class WebServer {
public:
  WebServer();
  ~WebServer();

  // Start Wi‑Fi AP and HTTP server. Returns true on success.
  bool start(const std::string &ssid, const std::string &password);
  void stop();

private:
  bool started_ = false;
  // std::shared_ptr<BleHidMouse> ble_;
};

} // namespace Hilton
