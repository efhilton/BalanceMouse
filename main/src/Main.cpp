#include <cstdio>

#include "FreeRTOSConfig.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <Finalizer.hpp>
#include <I2cManager.hpp>
#include <MouseEventDetector.hpp>
#include <Qmi8658C.hpp>
// #include <BleHidMouse.hpp>
#include <WebServer.hpp>
#include <any>
#include <memory>
#include <typeinfo>

static constexpr auto TAG{"BALANCE_BOARD"};

namespace Constants {
static constexpr auto I2C_NUM{I2C_NUM_0};
static constexpr auto RGB_CONTROL_PIN{GPIO_NUM_14};
static constexpr auto SDA_PIN{GPIO_NUM_11};
static constexpr auto SCL_PIN{GPIO_NUM_12};
static constexpr auto INT1_PIN{GPIO_NUM_10};
static constexpr auto INT2_PIN{GPIO_NUM_13};
} // namespace Constants

using BalanceBoardConfig = struct
{
    i2c_port_t i2cNum;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    gpio_num_t int1_pin;
    gpio_num_t int2_pin;
    gpio_num_t rgb_control_pin;
};

inline void onMouseData(
    const Hilton::MouseButtonStateMachine::State eventType, const Hilton::MouseEvent event, const std::any &userdata)
{
    // auto ble = std::any_cast<std::shared_ptr<Hilton::BleHidMouse>>(userdata);
    // ble->sendHidReport(event.buttons, event.deltaX, event.deltaY, event.wheel);
}

extern "C" void app_main(void)
{
    constexpr BalanceBoardConfig config{
        .i2cNum = Constants::I2C_NUM,
        .sda_pin = Constants::SDA_PIN,
        .scl_pin = Constants::SCL_PIN,
        .int1_pin = Constants::INT1_PIN,
        .int2_pin = Constants::INT2_PIN,
        .rgb_control_pin = Constants::RGB_CONTROL_PIN,
    };

    std::shared_ptr<Hilton::Qmi8658C> qmi8658c;
    std::unique_ptr<Hilton::I2cManager> i2cManager;
    std::shared_ptr<Hilton::MouseEventDetector> mouseEventDetector;
    // std::shared_ptr<Hilton::BleHidMouse> ble;
    std::unique_ptr<Hilton::WebServer> web;
    Hilton::Finalizer f{[&]() {
        if (web) {
            web->stop();
            web.reset();
        }
        // if (ble) {
        //   ble->end();
        //   ble.reset();
        // }
        if (mouseEventDetector) {
            mouseEventDetector.reset();
            mouseEventDetector = nullptr;
        }
        if (qmi8658c) {
            qmi8658c->onTerminate();
            qmi8658c.reset();
            qmi8658c = nullptr;
        }
        if (i2cManager) {
            i2cManager->terminate();
            i2cManager = nullptr;
        }
    }};

    i2cManager = std::make_unique<Hilton::I2cManager>(config.i2cNum, config.sda_pin, config.scl_pin);
    const esp_err_t err = i2cManager->initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C initialized in %d", config.i2cNum);

    ESP_LOGI(TAG, "Initializing QMI8658C");
    qmi8658c = std::make_shared<Hilton::Qmi8658C>(config.i2cNum);
    esp_err_t init_result = qmi8658c->initialize();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize QMI8658C: %s", esp_err_to_name(init_result));
        return;
    }
    ESP_LOGI(TAG, "QMI8658C initialized");

    ESP_LOGI(TAG, "Initializing Balance Board analytics");
    mouseEventDetector
        = std::make_shared<Hilton::MouseEventDetector>(qmi8658c, config.int1_pin, config.int2_pin);

    // ble = std::make_shared<Hilton::BleHidMouse>();
    // ble->begin("BalanceBoard Mouse");

    web = std::make_unique<Hilton::WebServer>();
    web->start("BalanceBoard-Setup", "balance123");

    mouseEventDetector->eventCallbackService
        .addCallback(Hilton::MouseButtonStateMachine::State::Idle, onMouseData, nullptr);
    mouseEventDetector->eventCallbackService
        .addCallback(Hilton::MouseButtonStateMachine::State::Pressed, onMouseData, nullptr);
    mouseEventDetector->eventCallbackService
        .addCallback(Hilton::MouseButtonStateMachine::State::Released, onMouseData, nullptr);
    mouseEventDetector->run();

    // Main task can do other things or just wait
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task heartbeat
    }
}