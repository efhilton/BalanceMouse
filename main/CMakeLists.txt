idf_component_register(
        SRCS
        "src/TinyUsbHID.cpp"
        "src/ChainOfResponsibility.cpp"
        "src/I2cManager.cpp"
        "src/Main.cpp"
        "src/MouseEventDetector.cpp"
        "src/Qmi8658C.cpp"
        "src/StringUtilities.cpp"

        INCLUDE_DIRS
        "."
        "./include"

        REQUIRES
        "driver"
        "esp_timer"
)
