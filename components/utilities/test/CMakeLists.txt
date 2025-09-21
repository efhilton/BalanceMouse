# Unit tests for the 'utilities' component
# These tests are built when using the ESP-IDF unit test app (idf.py -T ...)

idf_component_register(
    SRCS
        "test_QueuedNotificationService.cpp"
    PRIV_REQUIRES
        utilities
    REQUIRES
        unity
)
