idf_component_register(
        SRCS
        "src/BootButton.cpp"
        "src/I2cManager.cpp"
        "src/LedFeedbackTask.cpp"
        "src/Main.cpp"
        "src/MouseEventDetector.cpp"
        "src/Qmi8658C.cpp"

        INCLUDE_DIRS
        "."
        "./include"

        REQUIRES
        "driver"
        "esp_timer"
        "nvs_flash"
        "mouse"
        "gestures"
        "led_feedback"
        "neopixels"
        "ble_hid_mouse"
)
