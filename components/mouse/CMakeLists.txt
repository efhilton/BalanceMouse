# Portable mouse model + event-sink seam + HID report encoding. No ESP-IDF APIs, so it is
# also compiled by host_test/.
idf_component_register(
        SRCS "src/Mouse.cpp"

        INCLUDE_DIRS
        "include"
)
