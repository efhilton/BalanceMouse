// BOOT button press classification: debounce, short press on release, long press AT 3 s
// while still held (not on release), and no short press after a long press.
#include <ButtonPressDetector.hpp>
#include <cstdio>
#include <unity.h>
#include <vector>

using namespace Hilton;
using Event = ButtonPressDetector::Event;

void setUp() {
}

void tearDown() {
}

struct Fired {
    Event event;
    uint32_t atMs;
};

/** Feed a level for `durationMs` in 1 ms steps, collecting events. */
static void hold(ButtonPressDetector &d, const bool pressed, uint32_t &t, const uint32_t durationMs,
                 std::vector<Fired> &out) {
    for (const uint32_t end = t + durationMs; t < end; ++t) {
        const Event e = d.update(pressed, t);
        if (e != Event::None) out.push_back({e, t});
    }
}

static void test_short_press_fires_once_on_release() {
    ButtonPressDetector d;
    std::vector<Fired> fired;
    uint32_t t = 100;
    hold(d, false, t, 100, fired);
    hold(d, true, t, 250, fired);
    TEST_ASSERT_TRUE(d.isPressed());
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(fired.size())); // nothing while held
    const uint32_t releasedAt = t;
    hold(d, false, t, 200, fired);
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(fired.size()));
    TEST_ASSERT_TRUE(fired[0].event == Event::ShortPress);
    TEST_ASSERT_EQUAL_UINT32(releasedAt + ButtonPressDetector::DEFAULT_DEBOUNCE_MS, fired[0].atMs);
    TEST_ASSERT_FALSE(d.isPressed());
    TEST_ASSERT_FALSE(d.needsPolling());
}

static void test_contact_bounce_is_debounced() {
    ButtonPressDetector d;
    std::vector<Fired> fired;
    uint32_t t = 0;
    // Bouncy press: toggles every 3 ms for 20 ms, then solid.
    for (int i = 0; i < 7; ++i) hold(d, i % 2 == 0, t, 3, fired);
    hold(d, true, t, 300, fired);
    // Bouncy release.
    for (int i = 0; i < 7; ++i) hold(d, i % 2 != 0, t, 3, fired);
    hold(d, false, t, 100, fired);
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(fired.size()));
    TEST_ASSERT_TRUE(fired[0].event == Event::ShortPress);
}

static void test_glitch_shorter_than_debounce_is_ignored() {
    ButtonPressDetector d;
    std::vector<Fired> fired;
    uint32_t t = 0;
    hold(d, false, t, 50, fired);
    hold(d, true, t, ButtonPressDetector::DEFAULT_DEBOUNCE_MS - 1, fired);
    hold(d, false, t, 500, fired);
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(fired.size()));
    TEST_ASSERT_FALSE(d.isPressed());
}

static void test_long_press_fires_at_3s_while_still_held() {
    ButtonPressDetector d;
    std::vector<Fired> fired;
    uint32_t t = 1000;
    const uint32_t pressedAt = t;
    hold(d, true, t, ButtonPressDetector::DEFAULT_LONG_PRESS_MS - 1, fired);
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(fired.size())); // not a millisecond early
    hold(d, true, t, 2, fired);
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(fired.size()));
    TEST_ASSERT_TRUE(fired[0].event == Event::LongPress);
    TEST_ASSERT_EQUAL_UINT32(pressedAt + ButtonPressDetector::DEFAULT_LONG_PRESS_MS, fired[0].atMs);
    TEST_ASSERT_TRUE(d.isPressed()); // fired without waiting for release

    // Keep holding for a long time: still exactly one long press. Releasing adds nothing.
    hold(d, true, t, 10000, fired);
    hold(d, false, t, 500, fired);
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(fired.size()));

    // The next normal press is a short press again.
    hold(d, true, t, 200, fired);
    hold(d, false, t, 100, fired);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(fired.size()));
    TEST_ASSERT_TRUE(fired[1].event == Event::ShortPress);
}

static void test_release_just_before_3s_is_short() {
    ButtonPressDetector d;
    std::vector<Fired> fired;
    uint32_t t = 0;
    hold(d, true, t, ButtonPressDetector::DEFAULT_LONG_PRESS_MS - 10, fired);
    hold(d, false, t, 100, fired);
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(fired.size()));
    TEST_ASSERT_TRUE(fired[0].event == Event::ShortPress);
}

static void test_coarse_10ms_polling_like_the_device_task() {
    // The device task polls every 10 ms while held; long press must still land within one poll of 3 s.
    ButtonPressDetector d;
    Event got = Event::None;
    uint32_t firedAt = 0;
    for (uint32_t t = 0; t <= 4000 && got == Event::None; t += 10) {
        got = d.update(true, t);
        firedAt = t;
    }
    TEST_ASSERT_TRUE(got == Event::LongPress);
    TEST_ASSERT_UINT32_WITHIN(10, 3000, firedAt);
}

static void test_polling_hint() {
    ButtonPressDetector d;
    TEST_ASSERT_FALSE(d.needsPolling());
    d.update(true, 0); // edge seen, not yet debounced
    TEST_ASSERT_TRUE(d.needsPolling());
    d.update(true, 40);
    TEST_ASSERT_TRUE(d.needsPolling()); // held
    d.update(false, 100);
    d.update(false, 140);
    TEST_ASSERT_FALSE(d.needsPolling());
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0); // keep Unity output even if a sanitizer aborts the run
    UNITY_BEGIN();
    RUN_TEST(test_short_press_fires_once_on_release);
    RUN_TEST(test_contact_bounce_is_debounced);
    RUN_TEST(test_glitch_shorter_than_debounce_is_ignored);
    RUN_TEST(test_long_press_fires_at_3s_while_still_held);
    RUN_TEST(test_release_just_before_3s_is_short);
    RUN_TEST(test_coarse_10ms_polling_like_the_device_task);
    RUN_TEST(test_polling_hint);
    return UNITY_END();
}
