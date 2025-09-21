#include "unity.h"

#include <QueuedNotificationService.hpp>
#include <NotificationCallback.hpp>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <atomic>
#include <string>
#include <vector>
#include <functional>

using namespace Hilton;

namespace {

// Simple enum and payload type for testing
enum class TestKey { Alpha, Beta };
struct Payload {
    int value;
};

// Globals used by the test
static SemaphoreHandle_t g_sem = nullptr;
static std::atomic<int> g_callbackCount{0};
static std::atomic<int> g_lastValue{-1};

} // namespace

TEST_CASE("QueuedNotificationService basic callback invocation", "[queued]")
{
    // Arrange
    if (g_sem) {
        vSemaphoreDelete(g_sem);
    }
    g_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(g_sem);

    QueuedNotificationService<TestKey, Payload, std::any> service(4, "QNS_TEST");
    
    OnDataCallback<TestKey, Payload, std::any> cb = [](const TestKey &key, const Payload &data,  const std::any /*ignored*/) {
        g_lastValue = data.value;
        g_callbackCount++;
        xSemaphoreGive(g_sem);
    };

    const auto id = service.addCallback(TestKey::Alpha,cb, nullptr);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, static_cast<int>(id), "Failed to add callback");

    // Act
    service.notify(TestKey::Alpha, Payload{42});

    // Wait up to 1000ms for callback to run
    const auto taken = xSemaphoreTake(g_sem, pdMS_TO_TICKS(1000));

    // Assert
    TEST_ASSERT_EQUAL(pdTRUE, taken);
    TEST_ASSERT_EQUAL(1, g_callbackCount.load());
    TEST_ASSERT_EQUAL(42, g_lastValue.load());

    // Cleanup
    service.eraseCallback(id);
    vSemaphoreDelete(g_sem);
    g_sem = nullptr;
}
