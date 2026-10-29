# Host (x86/Linux) unit tests for the portable parts of the firmware:
#   components/mouse, components/gestures, components/led_feedback,
#   components/neopixels/include/LightPixel.hpp (50% intensity ceiling).
#
# Plain CMake + Unity (the copy bundled with ESP-IDF when IDF_PATH is set, otherwise
# fetched from GitHub). Built with AddressSanitizer + UBSan by default.
#
#   host_test/run_host_tests.sh          # runs everything inside the espressif/idf Docker image
#   cmake -S host_test -B build_host && cmake --build build_host && ctest --test-dir build_host
cmake_minimum_required(VERSION 3.16)
project(balance_board_host_tests C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(HOST_TEST_SANITIZERS "Build host tests with ASan + UBSan" ON)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(COMPONENTS ${REPO_ROOT}/components)

# ---- Unity ----
if (DEFINED ENV{IDF_PATH} AND EXISTS "$ENV{IDF_PATH}/components/unity/unity/src/unity.c")
    set(UNITY_SRC_DIR "$ENV{IDF_PATH}/components/unity/unity/src")
else ()
    include(FetchContent)
    FetchContent_Declare(unity_upstream
            GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
            GIT_TAG v2.6.0)
    FetchContent_Populate(unity_upstream)
    set(UNITY_SRC_DIR "${unity_upstream_SOURCE_DIR}/src")
endif ()
message(STATUS "Unity sources: ${UNITY_SRC_DIR}")

add_library(unity STATIC ${UNITY_SRC_DIR}/unity.c)
target_include_directories(unity PUBLIC ${UNITY_SRC_DIR})
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE)

set(WARNINGS -Wall -Wextra -Werror -Wno-missing-field-initializers)
set(SANITIZE_FLAGS "")
if (HOST_TEST_SANITIZERS)
    set(SANITIZE_FLAGS -fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer -fno-sanitize-recover=all)
endif ()

# ---- The portable firmware code under test (compiled exactly as on the device) ----
add_library(balance_core STATIC
        ${COMPONENTS}/mouse/src/Mouse.cpp
        ${COMPONENTS}/gestures/src/Calibration.cpp
        ${COMPONENTS}/gestures/src/GestureDetector.cpp
        ${COMPONENTS}/led_feedback/src/FeedbackRenderer.cpp
        ${COMPONENTS}/led_feedback/src/FeedbackState.cpp)
target_include_directories(balance_core PUBLIC
        ${COMPONENTS}/mouse/include
        ${COMPONENTS}/gestures/include
        ${COMPONENTS}/led_feedback/include
        ${COMPONENTS}/neopixels/include
        ${COMPONENTS}/exceptions/include)
target_compile_options(balance_core PRIVATE ${WARNINGS} ${SANITIZE_FLAGS})
target_link_options(balance_core PUBLIC ${SANITIZE_FLAGS})

enable_testing()

function(add_host_test name)
    add_executable(${name} ${name}.cpp)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${name} PRIVATE balance_core unity)
    target_compile_options(${name} PRIVATE ${WARNINGS} ${SANITIZE_FLAGS})
    target_link_options(${name} PRIVATE ${SANITIZE_FLAGS})
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES ENVIRONMENT
            "ASAN_OPTIONS=detect_stack_use_after_return=1:detect_leaks=1;UBSAN_OPTIONS=print_stacktrace=1")
endfunction()

add_host_test(test_bugfixes)
add_host_test(test_calibration)
add_host_test(test_gesture_events)
add_host_test(test_led_renderer)
add_host_test(test_boot_button)
