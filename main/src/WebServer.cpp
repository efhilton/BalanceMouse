#include "WebServer.hpp"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

namespace Hilton {

static const char *TAG_WS = "WebServer";

static std::shared_ptr<BleHidMouse> s_ble;

static const char index_html[] = R"HTML(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Balance Board Setup</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 2rem; }
      button { font-size: 1.2rem; padding: 0.6rem 1rem; }
    </style>
  </head>
  <body>
    <h1>Balance Board</h1>
    <p>Use this page to start Bluetooth pairing. Then pair from your PC's Bluetooth settings. The device will appear as a mouse.</p>
    <button onclick="fetch('/pair', {method: 'POST'}).then(()=>alert('Pairing/Advertising started')).catch(()=>alert('Failed'))">Start Bluetooth Pairing</button>
  </body>
</html>
)HTML";

static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static httpd_handle_t start_http_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG_WS, "Failed to start HTTP server");
    return nullptr;
  }

  httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &root_uri);

  ESP_LOGI(TAG_WS, "HTTP server started on port %d", config.server_port);

  return server;
}

WebServer::WebServer() = default;
WebServer::~WebServer() { stop(); }

bool WebServer::start(const std::string &ssid, const std::string &password) {
  if (started_) return true;

  // Init NVS, netif, event loop
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WS, "NVS init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t ap_config = {};
  strncpy(reinterpret_cast<char *>(ap_config.ap.ssid), ssid.c_str(), sizeof(ap_config.ap.ssid));
  ap_config.ap.ssid_len = ssid.size();
  strncpy(reinterpret_cast<char *>(ap_config.ap.password), password.c_str(), sizeof(ap_config.ap.password));
  ap_config.ap.max_connection = 4;
  ap_config.ap.channel = 1;
  ap_config.ap.authmode = password.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Start HTTP server
  auto server = start_http_server();
  if (!server) {
    return false;
  }

  started_ = true;
  ESP_LOGI(TAG_WS, "AP '%s' started. Connect and open http://192.168.4.1/", ssid.c_str());
  return true;
}

void WebServer::stop() {
  if (!started_) return;
  // We intentionally don't stop Wi‑Fi or http server here for simplicity in this minimal implementation.
  started_ = false;
}

} // namespace Hilton
