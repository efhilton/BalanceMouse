idf_component_register(
        SRCS
        src/led_strip_encoder.c
        src/PixelsController.cpp

        INCLUDE_DIRS
        "include"

        REQUIRES
        "driver"
        # Public headers (Color.hpp, LightConstants.hpp) include exception headers.
        "exceptions"
)
