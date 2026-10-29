# Portable LED feedback logic (pure renderer + state tracker). No ESP-IDF APIs, so it is
# also compiled by host_test/. The hardware write lives in main/src/LedFeedbackTask.cpp.
idf_component_register(
        SRCS
        "src/FeedbackRenderer.cpp"
        "src/FeedbackState.cpp"

        INCLUDE_DIRS
        "include"

        REQUIRES
        "mouse"
        "neopixels"
)
