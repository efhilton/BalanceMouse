# Portable gesture logic: no ESP-IDF APIs, so it is also compiled by host_test/.
idf_component_register(
        SRCS
        "src/Calibration.cpp"
        "src/GestureDetector.cpp"

        INCLUDE_DIRS
        "include"

        REQUIRES
        "mouse"
)
