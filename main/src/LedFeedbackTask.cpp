#include <LedFeedbackTask.hpp>
#include <MatrixLayout.hpp>
#include <esp_log.h>
#include <esp_timer.h>
#include <exception>

namespace Hilton {
    LedFeedbackTask::LedFeedbackTask(PixelsController &pixels, FeedbackState &state, Refresh refresh)
        : pixels(pixels), state(state), refresh(std::move(refresh)) {
    }

    esp_err_t LedFeedbackTask::start() {
        const BaseType_t created = xTaskCreate(&LedFeedbackTask::taskEntry, "led_feedback", 4096, this, 3, &task);
        return created == pdPASS ? ESP_OK : ESP_FAIL;
    }

    void LedFeedbackTask::taskEntry(void *arg) {
        static_cast<LedFeedbackTask *>(arg)->run();
    }

    void LedFeedbackTask::push(const Frame &frame) {
        for (int y = 0; y < Matrix::HEIGHT; ++y) {
            for (int x = 0; x < Matrix::WIDTH; ++x) {
                const RgbPixel &p = frame.at(x, y);
                LightPixel &px = pixels.getPixelById(Matrix::chainIndex(x, y));
                px.setColor(p.r, p.g, p.b, 0);
                px.setIntensity(p.intensity); // LightPixel clamps to the 50% ceiling as well
            }
        }
        pixels.write();
    }

    void LedFeedbackTask::run() {
        ESP_LOGI(TAG, "LED feedback task running at %lu ms/frame", static_cast<unsigned long>(FRAME_PERIOD_MS));
        Frame previous{};
        bool first = true;
        TickType_t lastWake = xTaskGetTickCount();
        while (true) {
            try {
                if (refresh) {
                    refresh(state);
                }
                const auto now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
                const Frame frame = FeedbackRenderer::render(state.snapshot(), now);
                if (first || frame != previous) {
                    push(frame);
                    previous = frame;
                    first = false;
                }
            } catch (const std::exception &e) {
                ESP_LOGE(TAG, "Frame failed: %s", e.what());
            }
            xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(FRAME_PERIOD_MS));
        }
    }
} // namespace Hilton
