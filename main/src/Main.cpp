#include <cstdio>

#include <any>
#include <I2cManager.hpp>
#include <memory>
#include <MouseEventDetector.hpp>
#include <Qmi8658C.hpp>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/projdefs.h>
#include <freertos/task.h>
#include <LedController.hpp>

static constexpr auto TAG{"BALANCE_BOARD"};

namespace Constants {
    static constexpr auto I2C_NUM{I2C_NUM_0}; // Changed from I2C_NUM_0 to just 0
    static constexpr auto LED_DIN_PIN{GPIO_NUM_14};
    static constexpr auto SDA_PIN{GPIO_NUM_11};
    static constexpr auto SCL_PIN{GPIO_NUM_12};
    static constexpr auto INT1_PIN{GPIO_NUM_10};
    static constexpr auto INT2_PIN{GPIO_NUM_13};
} // namespace Constants

using BalanceBoardConfig = struct {
    i2c_port_num_t i2cNum; // Changed type from i2c_port_t to int
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    gpio_num_t int1_pin;
    gpio_num_t int2_pin;
    gpio_num_t led_control_pin;
};

static OnSimpleCallback<std::shared_ptr<Hilton::Mouse> > onMouseData =
        [](const std::shared_ptr<Hilton::Mouse> &mouse, const std::any &ignored) {
    ESP_LOGI(TAG, "Mouse data received");
};

static OnSimpleCallback<std::shared_ptr<Hilton::Mouse> > controlLights = [
        ](const std::shared_ptr<Hilton::Mouse> &mouse, const std::any &ledControllerPtr) {
    ESP_LOGI(TAG, "Light Control data received");
};

extern "C" void app_main(void) {
    constexpr BalanceBoardConfig config{
        .i2cNum = Constants::I2C_NUM,
        .sda_pin = Constants::SDA_PIN,
        .scl_pin = Constants::SCL_PIN,
        .int1_pin = Constants::INT1_PIN,
        .int2_pin = Constants::INT2_PIN,
        .led_control_pin = Constants::LED_DIN_PIN,
    };

    auto i2CManager = std::make_unique<Hilton::I2CManager>(config.i2cNum, config.sda_pin, config.scl_pin);
    auto mouseEventDetector = std::make_shared<Hilton::MouseEventDetector>();
    auto ledController = std::make_shared<Hilton::LedController>(config.led_control_pin);
    auto mouse = std::make_shared<Hilton::Mouse>();

    const esp_err_t err = i2CManager->initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C initialized in I2C_NUM_%d", config.i2cNum);

    // Create QMI8658C with the bus handle from I2CManager
    auto qmi8658C = std::make_shared<Hilton::Qmi8658C>(i2CManager->getBusHandle());

    ESP_LOGI(TAG, "Initializing QMI8658C");
    esp_err_t init_result = qmi8658C->initialize(config.int1_pin, config.int2_pin);
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize QMI8658C: %s", esp_err_to_name(init_result));
        return;
    }
    ESP_LOGI(TAG, "QMI8658C initialized");

    // Register MouseEventDetector callback with sensor's queued notification service
    qmi8658C->dataService.addCallback(
        Hilton::Qmi8658C::QueueKey::DATA_READY,
        Hilton::MouseEventDetector::onSensorDataIsReady<Hilton::Qmi8658C::QueueKey, Hilton::Qmi8658C::DataPoint,
            std::any>,
        mouse);

    mouseEventDetector->simpleNotificationService.addCallback(controlLights, ledController);
    mouseEventDetector->simpleNotificationService.addCallback(onMouseData, nullptr);

    // Main task can do other things or just wait
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task heartbeat
    }
}
