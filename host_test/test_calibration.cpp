// Calibration: two independent, idle-only, slowly adapting baselines (tilt and yaw),
// boot-time seeding (after a warm-up, never from degenerate post-reset samples -- #1181),
// immediate re-zero, non-level resting stance, and the negative cases
// (held drag / sustained scroll / sustained movement must NOT be absorbed).
#include "test_support.hpp"

#include <ButtonPressDetector.hpp>
#include <Calibration.hpp>
#include <cmath>
#include <cstdint>
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

/** Feed the warm-up plus one seeding sample, i.e. "power on in this state". */
static void seed(Calibration &c, const MotionSample &s) {
    for (int i = 0; i <= Calibration::DEFAULT_WARMUP_SAMPLES; ++i) c.observe(s);
    TEST_ASSERT_TRUE(c.isSeeded());
}

static void test_seeds_both_baselines_after_warmup() {
    Calibration c;
    TEST_ASSERT_FALSE(c.isSeeded());
    for (int i = 0; i < Calibration::DEFAULT_WARMUP_SAMPLES; ++i) {
        c.observe(sampleOf(1.5f, -2.0f, 33.0f));
        TEST_ASSERT_FALSE(c.isSeeded());
        // Not seeded yet: calibrated output is held at zero, never "raw minus a zero baseline".
        TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltX());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltY());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedYaw());
    }
    c.observe(sampleOf(1.5f, -2.0f, 33.0f));
    TEST_ASSERT_TRUE(c.isSeeded());
    TEST_ASSERT_EQUAL_INT(Calibration::DEFAULT_WARMUP_SAMPLES, c.samplesRejectedBeforeSeed());
    TEST_ASSERT_EQUAL_FLOAT(1.5f, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, c.tiltBaselineY());
    TEST_ASSERT_EQUAL_FLOAT(33.0f, c.yawBaseline());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedYaw());
}

static void test_adapts_only_when_idle() {
    Calibration c;
    seed(c, sampleOf(0, 0, 0));
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
    seed(c, sampleOf(0, 0, 0));
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
    seed(c, sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, -1.0f, 10.0f));
    for (int i = 0; i < 250; ++i) c.adapt(0.02f, true); // 5 s
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, c.tiltBaselineX()); // 5 tau -> ~99.3%
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, c.tiltBaselineY());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.49f, c.yawBaseline()); // 0.05 tau -> ~4.9%

    // A yaw-only offset leaves the tilt baseline exactly where it was, and vice versa.
    Calibration d;
    seed(d, sampleOf(0.2f, 0.3f, 0));
    d.observe(sampleOf(0.2f, 0.3f, 12.0f));
    for (int i = 0; i < 500; ++i) d.adapt(0.02f, true);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, d.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(0.3f, d.tiltBaselineY());
    TEST_ASSERT_TRUE(d.yawBaseline() > 10.0f);
}

static void test_rezero_is_immediate() {
    Calibration c;
    seed(c, sampleOf(0, 0, 0));
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
    rig.boot(); // seed level
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


// ---------------- Regression (#1181): degenerate post-reset samples must not seed ----------------

// The real resting reading from the first hardware bring-up: AX=0x05C3, AY=0xF2FB at +/-2 g,
// converted exactly like Qmi8658C::populateRawAccelerometer (scale, then sign flip).
static constexpr float QMI_ACCEL_SCALE{2.0f * 9.81f / 32768.0f};
static constexpr float HW_REST_X{-static_cast<float>(static_cast<int16_t>(0x05C3)) * QMI_ACCEL_SCALE}; // -0.88 m/s^2
static constexpr float HW_REST_Y{-static_cast<float>(static_cast<int16_t>(0xF2FB)) * QMI_ACCEL_SCALE}; // +1.99 m/s^2
static const float HW_REST_Z{std::sqrt(9.81f * 9.81f - HW_REST_X * HW_REST_X - HW_REST_Y * HW_REST_Y)};
// The QMI8658 "0x84 in every data byte" garbage pattern the driver warns about, after the sign flip.
static constexpr float QMI_0x84_PATTERN{-static_cast<float>(static_cast<int16_t>(0x8484)) * QMI_ACCEL_SCALE}; // +18.9

static void test_zero_startup_samples_are_never_used_as_the_baseline() {
    // Exactly the hardware failure: data registers still at their reset value (all zero) for
    // MORE samples than the fixed warm-up, then real resting data. The zero samples must be
    // rejected (implausible gravity) and the seed must be the first real resting reading.
    Calibration c;
    constexpr int ZERO_SAMPLES{Calibration::DEFAULT_WARMUP_SAMPLES + 15};
    for (int i = 0; i < ZERO_SAMPLES; ++i) {
        MotionSample zero{};
        c.observe(zero);
        TEST_ASSERT_FALSE(c.isSeeded());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltX());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltY());
    }
    MotionSample rest{};
    rest.accel = {HW_REST_X, HW_REST_Y, HW_REST_Z};
    rest.yaw = 0.4f;
    c.observe(rest);
    TEST_ASSERT_TRUE(c.isSeeded());
    TEST_ASSERT_EQUAL_INT(ZERO_SAMPLES, c.samplesRejectedBeforeSeed());
    TEST_ASSERT_EQUAL_FLOAT(HW_REST_X, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(HW_REST_Y, c.tiltBaselineY());
    TEST_ASSERT_EQUAL_FLOAT(0.4f, c.yawBaseline());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltY());
}

static void test_plausible_resting_sample_window() {
    MotionSample s{};
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s)); // all-zero reset value
    s.accel = {HW_REST_X, HW_REST_Y, HW_REST_Z};
    TEST_ASSERT_TRUE(Calibration::isPlausibleRestingSample(s));
    s.accel = {0, 0, 9.81f};
    TEST_ASSERT_TRUE(Calibration::isPlausibleRestingSample(s));
    s.accel = {QMI_0x84_PATTERN, QMI_0x84_PATTERN, QMI_0x84_PATTERN}; // ~3.3 g
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
    s.accel = {0, 0, 0.4f * 9.81f};
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
    s.accel = {0, 0, 1.6f * 9.81f};
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
    s.accel = {std::nanf(""), 0, 9.81f};
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
    s.accel = {0, 0, 9.81f};
    s.yaw = std::nanf("");
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
}

/**
 * End to end through GestureDetector, reproducing the bring-up bug: the board rests untouched
 * at its real (non-level) tilt, but the IMU's first post-reset samples read garbage -- zeros,
 * then the 0x84 pattern -- for longer than the warm-up. With first-sample seeding the zero
 * sample became the baseline and the real rest read as a (-11, +25) lean forever: hypot ~28
 * report units, above the start threshold (13), so it never counted as idle and was never
 * adapted away. Now: no output at all during the garbage, the baseline is the real resting
 * reading, the board is idle, and it stays silent for a long untouched soak.
 */
static void test_garbage_startup_samples_then_real_rest_produces_no_phantom_movement() {
    Rig rig;
    // Phase 1: registers still at their reset value.
    rig.ax = 0;
    rig.ay = 0;
    rig.az = 0;
    for (int i = 0; i < Calibration::DEFAULT_WARMUP_SAMPLES + 5; ++i) {
        TEST_ASSERT_FALSE(rig.step());
    }
    // Phase 2: a burst of the 0x84 garbage pattern.
    rig.ax = rig.ay = rig.az = QMI_0x84_PATTERN;
    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_FALSE(rig.step());
    }
    TEST_ASSERT_FALSE(rig.calibration.isSeeded());
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(rig.sink.events.size())); // nothing emitted from garbage

    // Phase 3: real resting data (with a little sensor noise), board untouched, 60 s.
    uint32_t noise = 777;
    auto jitter = [&noise](const float amplitude) {
        noise = noise * 1103515245u + 12345u;
        return amplitude * (static_cast<float>((noise >> 16) & 0x7FFF) / 16383.5f - 1.0f);
    };
    for (int i = 0; i < 3000; ++i) {
        rig.ax = HW_REST_X + jitter(0.03f);
        rig.ay = HW_REST_Y + jitter(0.03f);
        rig.az = HW_REST_Z + jitter(0.03f);
        rig.step();
        TEST_ASSERT_TRUE(rig.calibration.isSeeded());
        TEST_ASSERT_TRUE_MESSAGE(rig.detector.wasIdle(), "untouched board at its real rest must count as idle");
    }
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Move));
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Scroll));
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(rig.sink.events.size()));
    TEST_ASSERT_FALSE(rig.mouse.isMoving());

    // Baseline is the real resting reading, not the zero / 0x84 transients.
    TEST_ASSERT_FLOAT_WITHIN(0.05f, HW_REST_X, rig.calibration.tiltBaselineX());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, HW_REST_Y, rig.calibration.tiltBaselineY());
    // Idle mouse output has converged to ~zero (well under one report unit = 0.077 m/s^2).
    rig.ax = HW_REST_X;
    rig.ay = HW_REST_Y;
    rig.az = HW_REST_Z;
    rig.run(10000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rig.detector.getCalibratedTiltX());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rig.detector.getCalibratedTiltY());
    TEST_ASSERT_EQUAL_INT(0, rig.mouse.getMovement().deltaX);
    TEST_ASSERT_EQUAL_INT(0, rig.mouse.getMovement().deltaY);

    // And the board still works as a mouse around that rest point.
    rig.ax = HW_REST_X + 2.0f;
    rig.run(200);
    TEST_ASSERT_TRUE(rig.mouse.isMoving());
    TEST_ASSERT_GREATER_THAN_INT(0, rig.sink.count(RecordedEvent::Type::Move));
}

static void test_rezero_during_warmup_does_not_seed_from_garbage() {
    // A BOOT short press while the IMU is still producing zeros must not snap the baseline to them.
    Rig rig;
    rig.ax = rig.ay = rig.az = 0;
    rig.step();
    rig.detector.requestRezero();
    rig.run(400); // 20 zero samples (> warm-up), request consumed meanwhile
    TEST_ASSERT_FALSE(rig.calibration.isSeeded());
    rig.ax = HW_REST_X;
    rig.ay = HW_REST_Y;
    rig.az = HW_REST_Z;
    rig.run(2000);
    TEST_ASSERT_TRUE(rig.calibration.isSeeded());
    TEST_ASSERT_EQUAL_FLOAT(HW_REST_X, rig.calibration.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(HW_REST_Y, rig.calibration.tiltBaselineY());
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(rig.sink.events.size()));
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0); // keep Unity output even if a sanitizer aborts the run
    UNITY_BEGIN();
    RUN_TEST(test_seeds_both_baselines_after_warmup);
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
    RUN_TEST(test_zero_startup_samples_are_never_used_as_the_baseline);
    RUN_TEST(test_plausible_resting_sample_window);
    RUN_TEST(test_garbage_startup_samples_then_real_rest_produces_no_phantom_movement);
    RUN_TEST(test_rezero_during_warmup_does_not_seed_from_garbage);
    return UNITY_END();
}
