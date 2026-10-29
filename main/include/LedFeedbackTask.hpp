#pragma once

#include <FeedbackRenderer.hpp>
#include <FeedbackState.hpp>
#include <PixelsController.hpp>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

namespace Hilton {
    /**
     * Periodic (25 Hz) FreeRTOS task: refreshes transport state into FeedbackState,
     * renders a frame with the PURE FeedbackRenderer, and only then touches hardware
     * (PixelsController) -- mapping logical (x, y) to chain index via MatrixLayout.
     * Unchanged frames are not re-sent.
     */
    class LedFeedbackTask {
    public:
        using Refresh = std::function<void(FeedbackState &)>;

        static constexpr auto TAG{"LED_FEEDBACK"};
        static constexpr uint32_t FRAME_PERIOD_MS{40};

        LedFeedbackTask(PixelsController &pixels, FeedbackState &state, Refresh refresh);

        esp_err_t start();

    private:
        static void taskEntry(void *arg);

        void run();

        void push(const Frame &frame);

        PixelsController &pixels;
        FeedbackState &state;
        Refresh refresh;
        TaskHandle_t task{nullptr};
    };
} // namespace Hilton
