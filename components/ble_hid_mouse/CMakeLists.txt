# BLE HID mouse transport (ESP-IDF only): NimBLE host + esp_hid (esp_hidd) HID-over-GATT.
idf_component_register(
        SRCS
        "src/BleHidMouse.cpp"

        INCLUDE_DIRS
        "include"

        REQUIRES
        "mouse"

        PRIV_REQUIRES
        "bt"
        "esp_hid"
        "esp_event"
)
