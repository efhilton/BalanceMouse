idf_component_register(
        SRCS
        "src/I2cManager.cpp"
        "src/Main.cpp"
        "src/MouseEventDetector.cpp"
        "src/Qmi8658C.cpp"
        "src/LedController.cpp"

        INCLUDE_DIRS
        "."
        "./include"

        REQUIRES
        "driver"
        "esp_timer"
        "nvs_flash"
        "esp_wifi"
        "esp_netif"
        "esp_event"
        "esp_http_server"
        "bt"
        "esp_hid"
)
