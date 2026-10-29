// Calibration: two independent, idle-only, slowly adapting baselines (tilt and yaw),
// boot-time seeding, immediate re-zero, non-level resting stance, and the negative cases
// (held drag / sustained scroll / sustained movement must NOT be absorbed).
#include "test_support.hpp"

#include <ButtonPressDetector.hpp>
#include <Calibration.hpp>
#include <cmath>
#include <cstdio>
#include <unity.h>

using namespace Hilton;
using namespace TestSupport;

void setUp() {
}

void tearDown() {
}

static MotionSample sampleOf(const float ax, const float ay, const float yaw, const uint32_t t = 0) {
    MotionSample s{};
    s.accel = {ax, ay, 9.8f};
    s.yaw = yaw;
    s.timestampMs = t;
    return s;
}

// ---------------- Calibration class on its own ----------------

static void test_first_sample_seeds_both_baselines() {
    Calibration c;
    TEST_ASSERT_FALSE(c.isSeeded());
    c.observe(sampleOf(1.5f, -2.0f, 33.0f));
    TEST_ASSERT_TRUE(c.isSeeded());
    TEST_ASSERT_EQUAL_FLOAT(1.5f, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, c.tiltBaselineY());
    TEST_ASSERT_EQUAL_FLOAT(33.0f, c.yawBaseline());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedYaw());
}

static void test_adapts_only_when_idle() {
    Calibration c;
    c.observe(sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, 1.0f, 10.0f));
    for (int i = 0; i < 1000; ++i) c.adapt(0.02f, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.yawBaseline());

    c.adapt(0.02f, true);
    TEST_ASSERT_TRUE(c.tiltBaselineX() > 0.0f && c.tiltBaselineX() < 0.01f); // one slow step
    TEST_ASSERT_TRUE(c.yawBaseline() > 0.0f && c.yawBaseline() < 0.1f);
}

static void test_time_constant_is_slow_and_exponential() {
    // tau = 5 s: after exactly one tau the baseline covers 1 - 1/e (63.2%) of a step.
    Calibration c(5.0f, 5.0f);
    c.observe(sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, 0, 10.0f));
    for (int i = 0; i < 250; ++i) c.adapt(0.02f, true); // 5 s at 50 Hz
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.632f, c.tiltBaselineX());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 6.32f, c.yawBaseline());
    // Zero / negative dt never moves it.
    const float before = c.tiltBaselineX();
    c.adapt(0.0f, true);
    c.adapt(-1.0f, true);
    TEST_ASSERT_EQUAL_FLOAT(before, c.tiltBaselineX());
}

static void test_tilt_and_yaw_baselines_are_independent() {
    // Different time constants prove they are separate filters, not one shared baseline.
    Calibration c(1.0f, 100.0f);
    c.observe(sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, -1.0f, 10.0f));
    for (int i = 0; i < 250; ++i) c.adapt(0.02f, true); // 5 s
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, c.tiltBaselineX()); // 5 tau -> ~99.3%
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, c.tiltBaselineY());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.49f, c.yawBaseline()); // 0.05 tau -> ~4.9%

    // A yaw-only offset leaves the tilt baseline exactly where it was, and vice versa.
    Calibration d;
    d.observe(sampleOf(0.2f, 0.3f, 0));
    d.observe(sampleOf(0.2f, 0.3f, 12.0f));
    for (int i = 0; i < 500; ++i) d.adapt(0.02f, true);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, d.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(0.3f, d.tiltBaselineY());
    TEST_ASSERT_TRUE(d.yawBaseline() > 10.0f);
}

static void test_rezero_is_immediate() {
    Calibration c;
    c.observe(sampleOf(0, 0, 0));
    c.observe(sampleOf(2.5f, -1.25f, -40.0f));
    c.rezero();
    TEST_ASSERT_EQUAL_FLOAT(2.5f, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(-1.25f, c.tiltBaselineY());
    TEST_ASSERT_EQUAL_FLOAT(-40.0f, c.yawBaseline());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedYaw());
}

// ---------------- Through GestureDetector (the real idle gating) ----------------

static void test_short_press_rezero_through_detector_stops_movement_and_scroll_immediately() {
    Rig rig;
    rig.step(); // seed level
    rig.ax = 4.0f;
    rig.run(200);
    TEST_ASSERT_TRUE(rig.mouse.isMoving());

    // BOOT short press -> ButtonPressDetector -> requestRezero (same wiring as Main.cpp).
    ButtonPressDetector button;
    uint32_t t = 0;
    ButtonPressDetector::Event ev = ButtonPressDetector::Event::None;
    for (; t < 200; t += 10) ev = button.update(true, t);
    TEST_ASSERT_TRUE(ev == ButtonPressDetector::Event::None);
    for (uint32_t end = t + 100; t < end && ev == ButtonPressDetector::Event::None; t += 10) ev = button.update(false, t);
    TEST_ASSERT_TRUE(ev == ButtonPressDetector::Event::ShortPress);
    rig.detector.requestRezero();

    rig.sink.clear();
    rig.step(); // the very next sample applies it
    TEST_ASSERT_EQUAL_FLOAT(4.0f, rig.calibration.tiltBaselineX());
    TEST_ASSERT_FALSE(rig.mouse.isMoving());
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Move));

    // Same for yaw / scroll.
    rig.yaw = 45.0f;
    rig.run(200);
    TEST_ASSERT_TRUE(rig.mouse.isScrollWheelActive());
    rig.detector.requestRezero();
    rig.sink.clear();
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(45.0f, rig.calibration.yawBaseline());
    TEST_ASSERT_FALSE(rig.mouse.isScrollWheelActive());
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Scroll));
}

static void test_power_on_in_a_non_level_stance_is_zero() {
    Rig rig;
    rig.ax = 3.0f; // ~18 degrees of roll at power-on: would be full-on movement without seeding
    rig.ay = -2.0f;
    rig.yaw = 25.0f;
    rig.run(2000);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Move));
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Scroll));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, rig.detector.getCalibratedTiltX());
}

/**
 * The ticket's edge case: a user whose natural resting stance is not level / not
 * pointing forward. Boot level, then settle into a constant offset (with sensor noise)
 * that sits below the movement/scroll START thresholds -- so nothing fires, but without
 * calibration the ranges would be asymmetric. The baselines must converge onto the
 * offset, stay there (no drift), and afterwards equal deliberate gestures in opposite
 * directions must produce equal-and-opposite output.
 */
static void test_non_level_resting_stance_converges_and_centres_output() {
    Rig rig;
    rig.run(1000); // boot level, seeded at 0

    constexpr float REST_X{0.70f}; // 0.70*12.96 = 9.1 units  (start threshold 13)
    constexpr float REST_Y{-0.60f}; // |(9.1, -7.8)| = 12.0 units -> still below 13
    constexpr float REST_YAW{6.0f}; // 6*127/60 = 12.7 units (start threshold 16)
    uint32_t noise = 12345;
    auto jitter = [&noise](const float amplitude) {
        noise = noise * 1103515245u + 12345u;
        return amplitude * (static_cast<float>((noise >> 16) & 0x7FFF) / 16383.5f - 1.0f);
    };

    for (int i = 0; i < 1500; ++i) { // 30 s standing in the new stance
        rig.ax = REST_X + jitter(0.03f);
        rig.ay = REST_Y + jitter(0.03f);
        rig.yaw = REST_YAW + jitter(0.2f);
        rig.step();
        TEST_ASSERT_TRUE_MESSAGE(rig.detector.wasIdle(), "resting stance should count as idle");
    }
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Move));
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Scroll));

    // Converged (30 s = 6 tau -> within ~0.25% of the offset, plus noise).
    TEST_ASSERT_FLOAT_WITHIN(0.02f, REST_X, rig.calibration.tiltBaselineX());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, REST_Y, rig.calibration.tiltBaselineY());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, REST_YAW, rig.calibration.yawBaseline());

    // No persistent drift: another 30 s at rest keeps it put.
    const float bx = rig.calibration.tiltBaselineX();
    const float byaw = rig.calibration.yawBaseline();
    rig.ax = REST_X;
    rig.ay = REST_Y;
    rig.yaw = REST_YAW;
    rig.run(30000);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, bx, rig.calibration.tiltBaselineX());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, byaw, rig.calibration.yawBaseline());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rig.detector.getCalibratedTiltX());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, rig.detector.getCalibratedYaw());

    // Symmetric movement: +/-2 m/s^2 of deliberate tilt around the new rest point.
    auto moveDx = [&rig](const float deltaX) {
        rig.ax = REST_X + deltaX;
        rig.ay = REST_Y;
        rig.sink.clear();
        rig.run(200);
        int dx = 0;
        for (const auto &e: rig.sink.events) {
            if (e.type == RecordedEvent::Type::Move) dx = e.dx;
        }
        rig.ax = REST_X;
        rig.run(500); // back to rest; movement stops
        return dx;
    };
    const int right = moveDx(+2.0f);
    const int left = moveDx(-2.0f);
    TEST_ASSERT_GREATER_THAN_INT(0, right);
    TEST_ASSERT_LESS_THAN_INT(0, left);
    TEST_ASSERT_INT_WITHIN(1, right, -left);

    // Symmetric scroll range: +/-15 degrees around the new heading.
    auto scrollFor = [&rig](const float deltaYaw) {
        rig.yaw = REST_YAW + deltaYaw;
        rig.sink.clear();
        rig.run(1000);
        const int total = rig.sink.scrollTotal();
        rig.yaw = REST_YAW;
        rig.run(500);
        return total;
    };
    const int up = scrollFor(+15.0f);
    const int down = scrollFor(-15.0f);
    TEST_ASSERT_GREATER_THAN_INT(0, up);
    TEST_ASSERT_LESS_THAN_INT(0, down);
    TEST_ASSERT_INT_WITHIN(1, up, -down);
}

// ---------------- Negative cases: never absorb deliberate input ----------------

static void test_held_drag_is_not_absorbed() {
    Rig rig;
    rig.run(500);
    rig.spike(-150.0f, 0.0f); // fast roll left -> Left button down
    TEST_ASSERT_TRUE(rig.mouse.isButtonPressed(MouseButtonStateMachine::Button::Left));

    // Drag: hold the button and lean -- first gently (below the movement start threshold, which
    // would be absorbed if the button weren't held), then hard -- for 20 s.
    rig.ax = 0.8f;
    rig.ay = 0.2f;
    rig.run(10000);
    rig.ax = 3.0f;
    rig.run(10000);
    TEST_ASSERT_TRUE(rig.mouse.isButtonPressed(MouseButtonStateMachine::Button::Left)); // slow lean doesn't release
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineY());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.yawBaseline());
    TEST_ASSERT_GREATER_THAN_INT(0, rig.sink.count(RecordedEvent::Type::Move)); // the drag did move the cursor
}

static void test_sustained_scroll_is_not_absorbed() {
    Rig rig;
    rig.run(500);
    rig.yaw = 20.0f; // 42 units: scrolling
    rig.run(1000);
    const int firstSecond = rig.sink.scrollTotal();
    rig.run(18000);
    rig.sink.clear();
    rig.run(1000);
    const int lastSecond = rig.sink.scrollTotal();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.yawBaseline());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineX());
    TEST_ASSERT_TRUE(rig.mouse.isScrollWheelActive());
    TEST_ASSERT_GREATER_THAN_INT(0, firstSecond);
    TEST_ASSERT_INT_WITHIN(1, firstSecond, lastSecond); // still scrolling at the same rate after 20 s
}

static void test_sustained_movement_is_not_absorbed() {
    Rig rig;
    rig.run(500);
    rig.ax = 2.0f;
    rig.run(20000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineX());
    TEST_ASSERT_TRUE(rig.mouse.isMoving());
}

static void test_idle_between_gestures_resumes_adaptation() {
    Rig rig;
    rig.run(500);
    rig.spike(-150.0f, 0.0f); // left down
    rig.ax = 0.5f;
    rig.run(2000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineX());
    rig.run(400); // > 300 ms lockout
    rig.spike(+150.0f, 0.0f); // left up
    TEST_ASSERT_FALSE(rig.mouse.isButtonPressed(MouseButtonStateMachine::Button::Left));
    rig.run(5000);
    TEST_ASSERT_TRUE(rig.calibration.tiltBaselineX() > 0.25f); // now idle -> adapting toward 0.5
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0); // keep Unity output even if a sanitizer aborts the run
    UNITY_BEGIN();
    RUN_TEST(test_first_sample_seeds_both_baselines);
    RUN_TEST(test_adapts_only_when_idle);
    RUN_TEST(test_time_constant_is_slow_and_exponential);
    RUN_TEST(test_tilt_and_yaw_baselines_are_independent);
    RUN_TEST(test_rezero_is_immediate);
    RUN_TEST(test_short_press_rezero_through_detector_stops_movement_and_scroll_immediately);
    RUN_TEST(test_power_on_in_a_non_level_stance_is_zero);
    RUN_TEST(test_non_level_resting_stance_converges_and_centres_output);
    RUN_TEST(test_held_drag_is_not_absorbed);
    RUN_TEST(test_sustained_scroll_is_not_absorbed);
    RUN_TEST(test_sustained_movement_is_not_absorbed);
    RUN_TEST(test_idle_between_gestures_resumes_adaptation);
    return UNITY_END();
}
