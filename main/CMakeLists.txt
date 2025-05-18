idf_component_register(
        SRCS
        "src/Main.cpp"
        "src/Qmi8658C.cpp"

        INCLUDE_DIRS
        "."
        "./include"

        REQUIRES
        "driver"
        "esp_timer"
)
