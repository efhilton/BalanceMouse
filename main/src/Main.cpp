#include <BleHidMouse.hpp>
#include <BootButton.hpp>
#include <FeedbackState.hpp>
#include <I2cManager.hpp>
#include <LedFeedbackTask.hpp>
#include <MatrixLayout.hpp>
#include <MouseEventDetector.hpp>
#include <MouseEventSink.hpp>
#include <PixelsController.hpp>
#include <Qmi8658C.hpp>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/projdefs.h>
#include <freertos/task.h>
#include <memory>
#include <nvs_flash.h>

static constexpr auto TAG{"BALANCE_BOARD"};

namespace Constants {
    static constexpr auto I2C_NUM{I2C_NUM_0};
    static constexpr auto LED_DIN_PIN{GPIO_NUM_14};
    static constexpr auto SDA_PIN{GPIO_NUM_11};
    static constexpr auto SCL_PIN{GPIO_NUM_12};
    static constexpr auto INT1_PIN{GPIO_NUM_10};
    static constexpr auto INT2_PIN{GPIO_NUM_13};
    static constexpr auto BOOT_BUTTON_PIN{GPIO_NUM_0};
    static constexpr auto BLE_DEVICE_NAME{"BalanceMouse"};
} // namespace Constants

using BalanceBoardConfig = struct {
    i2c_port_num_t i2cNum;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    gpio_num_t int1_pin;
    gpio_num_t int2_pin;
    gpio_num_t led_control_pin;
};

static OnSimpleCallback<Hilton::Mouse *> onMouseData =
        [](const Hilton::Mouse *mouse, const void * /*ignored*/) {
    mouse->printStatus();
};

static uint32_t nowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

static esp_err_t initNvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

extern "C" void app_main(void) {
    constexpr BalanceBoardConfig config{
        .i2cNum = Constants::I2C_NUM,
        .sda_pin = Constants::SDA_PIN,
        .scl_pin = Constants::SCL_PIN,
        .int1_pin = Constants::INT1_PIN,
        .int2_pin = Constants::INT2_PIN,
        .led_control_pin = Constants::LED_DIN_PIN,
    };

    // Everything below is referenced by long-running tasks, so it must outlive app_main's
    // scope; app_main never returns (see the heartbeat loop at the end).

    // NVS holds the BLE bonds (CONFIG_BT_NIMBLE_NVS_PERSIST).
    ESP_ERROR_CHECK(initNvs());

    // --- LED matrix + feedback state ---
    static Hilton::FeedbackState feedback{nowMs};
    static Hilton::PixelsController pixels{
        Hilton::PixelsController::PixelsConfig{
            .controlPin = config.led_control_pin,
            .totalNumberOfPixels = Hilton::Matrix::PIXEL_COUNT,
            .targetDevice = Hilton::PixelsController::WS2812B,
            // Waveshare's examples drive this board with NEO_RGB. If red/green look swapped, use GRB.
            .colorOrderPerPixel = Hilton::Lights::Pixel::ColorOrder::RGB,
        }
    };
    const bool ledsOk = pixels.initialize() == ESP_OK;

    // --- BLE HID transport ---
    auto &ble = Hilton::BleHidMouse::instance();
    const esp_err_t bleErr = ble.start(Constants::BLE_DEVICE_NAME);
    if (bleErr != ESP_OK) {
        ESP_LOGE(TAG, "BLE HID failed to start: %s (continuing without it)", esp_err_to_name(bleErr));
    }

    static Hilton::LedFeedbackTask ledTask{
        pixels, feedback,
        [](Hilton::FeedbackState &state) {
            const auto &b = Hilton::BleHidMouse::instance();
            state.updateConnection(b.getPhase(), b.getPairingStatus());
        }
    };
    if (ledsOk && ledTask.start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start LED feedback task");
    }

    // --- Gesture events fan out to the HID transport and the LED feedback ---
    static Hilton::MouseEventFanout events;
    events.addSink(&ble);
    events.addSink(&feedback);

    static Hilton::Mouse mouse;
    static Hilton::MouseEventDetector mouseEventDetector{mouse, &events, &feedback};

    const uint32_t id2 = mouseEventDetector.simpleNotificationService.addCallback(onMouseData, nullptr);
    ESP_LOGI(TAG, "Registered mouseEventDetector status callback with id %u", id2);

    // --- BOOT button: short press re-zeroes calibration, 3 s hold forgets the bond and re-pairs ---
    static Hilton::BootButton bootButton{
        Constants::BOOT_BUTTON_PIN,
        [] { mouseEventDetector.requestRezero(); },
        [] { Hilton::BleHidMouse::instance().forgetBondsAndRestartPairing(); },
    };
    if (bootButton.start() != ESP_OK) {
        ESP_LOGE(TAG, "BOOT button setup failed");
    }

    // --- IMU ---
    static Hilton::I2CManager i2CManager{config.i2cNum, config.sda_pin, config.scl_pin};
    esp_err_t err = i2CManager.initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "I2C initialized in I2C_NUM_%d", config.i2cNum);

        static Hilton::Qmi8658C qmi8658C{i2CManager.getBusHandle()};
        ESP_LOGI(TAG, "Initializing QMI8658C");
        err = qmi8658C.initialize();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize QMI8658C: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "QMI8658C initialized");
            auto onSensorData = [](const Hilton::Qmi8658C::DataPoint &data, const void * /*ignored*/) {
                mouseEventDetector.onSensorDataIsReady(data);
            };
            const uint32_t id = qmi8658C.simpleNotificationService.addCallback(onSensorData, nullptr);
            ESP_LOGI(TAG, "Registered QMI8658 callback with id %u", id);

#ifdef DO_DEBUG
            auto printData = [](const Hilton::Qmi8658C::DataPoint &data, const void * /*ignored*/) {
                ESP_LOGI(TAG, "Raw data: A[%5.1f %5.1f %5.1f] G[%5.1f %5.1f %5.1f]",
                         data.rawAccel.x, data.rawAccel.y, data.rawAccel.z,
                         data.rawGyro.x, data.rawGyro.y, data.rawGyro.z);
            };
            const uint32_t id4 = qmi8658C.simpleNotificationService.addCallback(printData, nullptr);
            ESP_LOGI(TAG, "Registered print raw with id %u", id4);
#endif
        }
    }

    // Main task heartbeat; never return (tasks above reference the statics).
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
