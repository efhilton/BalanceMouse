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

/**
 * Feed the warm-up plus enough settling samples to seed, i.e. "power on in this state".
 * Seeding averages Calibration::SETTLING_SAMPLES plausible samples, not just one (#1181).
 */
static void seed(Calibration &c, const MotionSample &s) {
    for (int i = 0; i < Calibration::DEFAULT_WARMUP_SAMPLES + Calibration::SETTLING_SAMPLES; ++i) c.observe(s);
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
    // Seeding now averages SETTLING_SAMPLES plausible samples instead of taking just one
    // (#1181) -- still not seeded until the last of them arrives.
    for (int i = 0; i < Calibration::SETTLING_SAMPLES - 1; ++i) {
        c.observe(sampleOf(1.5f, -2.0f, 33.0f));
        TEST_ASSERT_FALSE(c.isSeeded());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, c.calibratedTiltX());
    }
    c.observe(sampleOf(1.5f, -2.0f, 33.0f));
    TEST_ASSERT_TRUE(c.isSeeded());
    TEST_ASSERT_EQUAL_INT(Calibration::DEFAULT_WARMUP_SAMPLES, c.samplesRejectedBeforeSeed());
    // Every averaged sample was identical, so the average is exact, not just close.
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
    for (int i = 0; i < 1000; ++i) c.adapt(0.02f, false, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.yawBaseline());

    c.adapt(0.02f, true, true);
    TEST_ASSERT_TRUE(c.tiltBaselineX() > 0.0f && c.tiltBaselineX() < 0.01f); // one slow step
    TEST_ASSERT_TRUE(c.yawBaseline() > 0.0f && c.yawBaseline() < 0.1f);
}

static void test_time_constant_is_slow_and_exponential() {
    // tau = 5 s: after exactly one tau the baseline covers 1 - 1/e (63.2%) of a step.
    Calibration c(5.0f, 5.0f);
    seed(c, sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, 0, 10.0f));
    for (int i = 0; i < 250; ++i) c.adapt(0.02f, true, true); // 5 s at 50 Hz
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.632f, c.tiltBaselineX());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 6.32f, c.yawBaseline());
    // Zero / negative dt never moves it.
    const float before = c.tiltBaselineX();
    c.adapt(0.0f, true, true);
    c.adapt(-1.0f, true, true);
    TEST_ASSERT_EQUAL_FLOAT(before, c.tiltBaselineX());
}

/**
 * adapt()'s two idle flags are independently honoured, not just independently NAMED (#1181,
 * Edgar's real-hardware find motivated splitting them -- see Calibration::adapt()'s doc
 * comment). Positive/negative on each flag, checked against the OTHER baseline too so a bug
 * that silently applies one flag to both filters would be caught.
 */
static void test_tilt_idle_and_yaw_idle_flags_are_independently_honoured() {
    Calibration c;
    seed(c, sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, 1.0f, 10.0f));

    // tiltIdle only: tilt moves, yaw does not.
    c.adapt(0.02f, true, false);
    TEST_ASSERT_TRUE(c.tiltBaselineX() > 0.0f);
    TEST_ASSERT_TRUE(c.tiltBaselineY() > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.yawBaseline());

    // yawIdle only: yaw moves, tilt stays exactly where tiltIdle left it.
    const float tiltXAfterFirstStep = c.tiltBaselineX();
    const float tiltYAfterFirstStep = c.tiltBaselineY();
    c.adapt(0.02f, false, true);
    TEST_ASSERT_EQUAL_FLOAT(tiltXAfterFirstStep, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(tiltYAfterFirstStep, c.tiltBaselineY());
    TEST_ASSERT_TRUE(c.yawBaseline() > 0.0f);

    // Neither: nothing moves.
    const float tiltXBefore = c.tiltBaselineX();
    const float yawBefore = c.yawBaseline();
    c.adapt(0.02f, false, false);
    TEST_ASSERT_EQUAL_FLOAT(tiltXBefore, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(yawBefore, c.yawBaseline());
}

/**
 * Edgar (#1181, 2026-11-02): "within 10 seconds, that direction becomes our new zero." With the
 * DEFAULT yaw time constant (3 s), an idle yaw step must be >= 95% absorbed after 10 s
 * (1 - e^(-10/3) ~= 96.4%), and not yet before ~9 s (3*tau = 95% at 9 s) -- i.e. it is the time
 * constant doing this, not some faster snap. Tilt keeps its own, unchanged 5 s constant.
 */
static void test_default_yaw_ema_converges_within_10_seconds_while_idle() {
    Calibration c;
    seed(c, sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, 0, 8.0f));
    for (int i = 0; i < 400; ++i) c.adapt(0.02f, true, true); // 8 s at 50 Hz
    TEST_ASSERT_TRUE_MESSAGE(c.yawBaseline() < 0.95f * 8.0f, "8 s: should not be 95% converged yet");
    for (int i = 0; i < 100; ++i) c.adapt(0.02f, true, true); // -> 10 s
    TEST_ASSERT_TRUE_MESSAGE(c.yawBaseline() >= 0.95f * 8.0f, "10 s: yaw must be >= 95% converged");
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 8.0f * (1.0f - std::exp(-10.0f / 3.0f)), c.yawBaseline());
    // Tilt is on the unchanged 5 s constant: 1 - e^-2 ~= 86.5% at 10 s.
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.0f - std::exp(-2.0f), c.tiltBaselineX());
}

/**
 * Same requirement end-to-end through GestureDetector's real yawIdle gating: the board turned to
 * a new heading below the scroll start threshold (8 deg = 12.7 units < 15) and left alone. Within
 * 10 s that heading is the new zero, with no scroll emitted along the way.
 */
static void test_new_heading_becomes_zero_within_10_seconds_through_detector() {
    Rig rig;
    rig.boot();
    constexpr float NEW_HEADING{8.0f};
    rig.yaw = NEW_HEADING;
    rig.run(10000);
    TEST_ASSERT_FALSE(rig.mouse.isScrollWheelActive());
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Scroll));
    TEST_ASSERT_TRUE_MESSAGE(rig.calibration.yawBaseline() >= 0.95f * NEW_HEADING,
                             "yaw baseline must be >= 95% of the new heading after 10 s idle");
    TEST_ASSERT_TRUE(std::abs(rig.detector.getCalibratedYaw()) <= 0.05f * NEW_HEADING);
}

static void test_tilt_and_yaw_baselines_are_independent() {
    // Different time constants prove they are separate filters, not one shared baseline.
    Calibration c(1.0f, 100.0f);
    seed(c, sampleOf(0, 0, 0));
    c.observe(sampleOf(1.0f, -1.0f, 10.0f));
    for (int i = 0; i < 250; ++i) c.adapt(0.02f, true, true); // 5 s
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, c.tiltBaselineX()); // 5 tau -> ~99.3%
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, c.tiltBaselineY());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.49f, c.yawBaseline()); // 0.05 tau -> ~4.9%

    // A yaw-only offset leaves the tilt baseline exactly where it was, and vice versa.
    Calibration d;
    seed(d, sampleOf(0.2f, 0.3f, 0));
    d.observe(sampleOf(0.2f, 0.3f, 12.0f));
    for (int i = 0; i < 500; ++i) d.adapt(0.02f, true, true);
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
    rig.ax = 3.0f; // ~18 degrees of lean on the X (left/right) channel at power-on: would be
                   // full-on movement without seeding
    rig.ay = -2.0f;
    rig.yaw = 25.0f;
    rig.boot(); // seeding takes longer now (averages SETTLING_SAMPLES, #1181) but stays silent
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
    rig.boot(); // boot level, seeded at 0

    constexpr float REST_X{0.70f}; // 0.70*12.96 = 9.1 units  (TILT_IDLE_MAGNITUDE_THRESHOLD 13)
    constexpr float REST_Y{-0.60f}; // |(9.1, -7.8)| = 12.0 units -> still below 13
    constexpr float REST_YAW{6.0f}; // 6*127/80 = 9.5 units (start threshold 15, #1181)
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
    rig.boot();
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
    rig.boot();
    rig.yaw = 20.0f; // 31 units (SCROLL_SCALE = 127/80, #1181): scrolling
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
    rig.boot();
    rig.ax = 2.0f;
    rig.run(20000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineX());
    TEST_ASSERT_TRUE(rig.mouse.isMoving());
}

static void test_idle_between_gestures_resumes_adaptation() {
    Rig rig;
    rig.boot();
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
    // Seeding now averages SETTLING_SAMPLES plausible samples instead of taking just one (#1181).
    for (int i = 0; i < Calibration::SETTLING_SAMPLES - 1; ++i) {
        c.observe(rest);
        TEST_ASSERT_FALSE(c.isSeeded());
    }
    c.observe(rest);
    TEST_ASSERT_TRUE(c.isSeeded());
    TEST_ASSERT_EQUAL_INT(ZERO_SAMPLES, c.samplesRejectedBeforeSeed());
    // WITHIN, not EQUAL: HW_REST_X/Y are not exact binary fractions, so averaging 500 copies
    // (#1181) accumulates a little float-summation rounding (same reasoning as below).
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, HW_REST_X, c.tiltBaselineX());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, HW_REST_Y, c.tiltBaselineY());
    TEST_ASSERT_EQUAL_FLOAT(0.4f, c.yawBaseline());
    // Within float-summation noise of zero, not bit-exact: averaging 500 samples (#1181)
    // accumulates a little float rounding that a single-sample seed never had to deal with.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.calibratedTiltX());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.calibratedTiltY());
}

/**
 * Scope clarification (#1181, Edgar/Maria, 2026-11-02): the settling average must cover all
 * three axes -- X and Y (tilt, cursor movement) AND Z (yaw, scroll) -- not just tilt. The
 * implementation already satisfies this by construction: tiltX/tiltY/yaw share one settling
 * accumulator inside a single Calibration::observe() call (see settleSumTiltX/Y/Yaw in
 * Calibration.hpp), so there is no code path that seeds tilt without also seeding yaw the same
 * way. But every settling test above happens to hold yaw CONSTANT across the window, which
 * would pass identically even if yaw were (bug) seeded from one sample instead of averaged.
 * This test varies yaw sample-by-sample across the whole window and checks the seeded baseline
 * lands on the arithmetic mean of what was fed -- not the first sample, not the last -- proving
 * yaw genuinely goes through the same running average as tilt, not a shortcut.
 */
static void test_settling_average_includes_varying_yaw_not_just_tilt() {
    Calibration c;
    for (int i = 0; i < Calibration::DEFAULT_WARMUP_SAMPLES; ++i) c.observe(sampleOf(0, 0, 0));
    TEST_ASSERT_FALSE(c.isSeeded());

    double yawSum = 0.0; // double accumulator for a precise reference mean, independent of the
                          // float summation under test
    constexpr float YAW_START{10.0f};
    constexpr float YAW_STEP{0.01f};
    for (int i = 0; i < Calibration::SETTLING_SAMPLES; ++i) {
        const float yaw = YAW_START + YAW_STEP * static_cast<float>(i); // ramps across the window
        yawSum += yaw;
        c.observe(sampleOf(1.2f, -0.8f, yaw));
    }
    TEST_ASSERT_TRUE(c.isSeeded());
    const float expectedYaw = static_cast<float>(yawSum / Calibration::SETTLING_SAMPLES);
    const float lastYaw = YAW_START + YAW_STEP * static_cast<float>(Calibration::SETTLING_SAMPLES - 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expectedYaw, c.yawBaseline());
    // Not the first sample fed, and not the last -- the true mean of the whole ramp.
    TEST_ASSERT_TRUE(c.yawBaseline() > YAW_START + 0.1f);
    TEST_ASSERT_TRUE(c.yawBaseline() < lastYaw - 0.1f);
    // Tilt (held constant here) still seeds correctly alongside it.
    TEST_ASSERT_EQUAL_FLOAT(1.2f, c.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(-0.8f, c.tiltBaselineY());
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

    // Gravity-plausible but still rotating (e.g. being carried/rotated during power-on or BLE
    // pairing) must NOT count as a resting sample, even though the magnitude alone looks fine.
    s.accel = {0, 0, 9.81f};
    s.yaw = 0.0f;
    s.gyro = {0, 0, 0};
    TEST_ASSERT_TRUE(Calibration::isPlausibleRestingSample(s));
    s.gyro = {40.0f, 0, 0};
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
    s.gyro = {0, 0, 40.0f};
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
    s.gyro = {Calibration::MAX_SEED_GYRO_DPS - 0.1f, 0, 0}; // just under the limit: still resting
    TEST_ASSERT_TRUE(Calibration::isPlausibleRestingSample(s));
    s.gyro = {Calibration::MAX_SEED_GYRO_DPS + 0.1f, 0, 0}; // just over: rejected
    TEST_ASSERT_FALSE(Calibration::isPlausibleRestingSample(s));
}

/**
 * Real hardware finding (#1181, Edgar's BLE/LED bring-up pass): the board was still being
 * handled (rotated into position for BLE pairing) when its warm-up elapsed. The old seeding
 * rule (gravity magnitude alone) happily seeded from that mid-handling sample, baking in an
 * angle well away from the board's real resting stance. Because that offset read as larger
 * than the movement START threshold, it was deliberately never adapted away (the same
 * protection that keeps a genuine sustained drag/lean from being absorbed, see
 * test_sustained_movement_is_not_absorbed) -- confirmed live: the board sat forever reporting
 * a large constant deltaX/deltaY and an off-centre LED ball, with no automatic recovery.
 *
 * Fix under test: seeding now also requires the gyro to show no rotation, so a mid-handling
 * sample is rejected (not counted toward the SETTLING_SAMPLES average, see Calibration's
 * class doc, #1181) and the baseline is only averaged from samples once the board is
 * genuinely still.
 */
static void test_seeding_while_still_being_handled_is_rejected_until_genuinely_still() {
    Rig rig;
    // Phase 1: warm-up elapses while the board is being rotated into its final position for
    // pairing -- gravity-plausible at every instant, but actively rotating.
    for (int i = 0; i <= Calibration::DEFAULT_WARMUP_SAMPLES; ++i) {
        rig.ax = 3.0f; // a gravity-plausible but very much NOT-final angle
        rig.ay = -2.0f;
        rig.gx = 60.0f; // clearly rotating
        rig.step();
        TEST_ASSERT_FALSE_MESSAGE(rig.calibration.isSeeded(), "must not seed while still rotating");
    }
    // A couple more seconds of continued handling: still must not seed, and none of it counts
    // toward settling (every one of these samples fails the gyro check).
    for (int i = 0; i < 100; ++i) {
        rig.ax = 1.0f + 0.02f * static_cast<float>(i); // angle still changing
        rig.ay = -1.0f;
        rig.gx = 30.0f;
        rig.step();
        TEST_ASSERT_FALSE(rig.calibration.isSeeded());
        TEST_ASSERT_EQUAL_INT(0, rig.calibration.settledSampleCount());
    }
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(rig.sink.events.size())); // nothing emitted while unseeded

    // Phase 2: the board is set down and comes to rest at ITS real stance -- gyro settles to ~0.
    // None of Phase 1's rotating samples contributed, so the average is exactly the real rest.
    rig.ax = 0.70f;
    rig.ay = -0.60f;
    rig.gx = 0.0f;
    for (int i = 0; i < Calibration::SETTLING_SAMPLES - 1; ++i) {
        rig.step();
        TEST_ASSERT_FALSE_MESSAGE(rig.calibration.isSeeded(), "settling takes SETTLING_SAMPLES, not one");
    }
    rig.step();
    TEST_ASSERT_TRUE_MESSAGE(rig.calibration.isSeeded(), "must seed once genuinely still for long enough");
    TEST_ASSERT_EQUAL_FLOAT(0.70f, rig.calibration.tiltBaselineX());
    TEST_ASSERT_EQUAL_FLOAT(-0.60f, rig.calibration.tiltBaselineY());

    // The real stance now reads as centred, immediately and from then on -- no permanent
    // phantom movement, no off-centre idle display.
    rig.run(2000);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, rig.detector.getCalibratedTiltX());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, rig.detector.getCalibratedTiltY());
    TEST_ASSERT_EQUAL_INT(0, rig.mouse.getMovement().deltaX);
    TEST_ASSERT_EQUAL_INT(0, rig.mouse.getMovement().deltaY);
    TEST_ASSERT_FALSE(rig.mouse.isMoving());
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(RecordedEvent::Type::Move));
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
    // Seeding now averages SETTLING_SAMPLES plausible samples instead of taking just one
    // (#1181) -- so it is not seeded from the very first sample of this phase; only assert
    // wasIdle() once it actually is.
    for (int i = 0; i < Calibration::SETTLING_SAMPLES + 2500; ++i) {
        rig.ax = HW_REST_X + jitter(0.03f);
        rig.ay = HW_REST_Y + jitter(0.03f);
        rig.az = HW_REST_Z + jitter(0.03f);
        rig.step();
        if (rig.calibration.isSeeded()) {
            TEST_ASSERT_TRUE_MESSAGE(rig.detector.wasIdle(), "untouched board at its real rest must count as idle");
        }
    }
    TEST_ASSERT_TRUE(rig.calibration.isSeeded());
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

/**
 * Real-hardware bug (#1181, Edgar): with the board sitting completely untouched, the live
 * serial VERIFY log showed calibratedYaw drifting ~0.5-0.6 deg/s on its own -- gyro Z has no
 * accelerometer-style absolute reference, so its integration drifts unboundedly even at rest.
 * Once that drift alone pushed calibratedYaw past the scroll ENGAGE threshold, the scroll
 * wheel latched active and, under the OLD code (one shared `idle` flag requiring
 * !mouse.isScrollWheelActive()), yaw-baseline adaptation was blocked from then on -- a one-way
 * ratchet that never recovers by itself.
 *
 * The fix is NOT a gradual EMA correction while stuck -- that was tried first and discarded: no
 * time constant is simultaneously negligible over test_sustained_scroll_is_not_absorbed's 20 s
 * window and fast enough to be a useful recovery (see Calibration::adapt()'s doc comment). It
 * is a one-time, decisive snap (Calibration::snapYawToRaw()) fired once "wheel active, nothing
 * else going on, gyro quiet" has persisted continuously for longer than
 * GestureDetector::STUCK_SCROLL_RECOVERY_S (30 s) -- see its own doc comment. This reproduces
 * the drift-into-saturation scenario directly (not via real integration, which isn't modelled
 * here -- `rig.yaw` is just set to a drifted value, the same as if gyro integration had
 * produced it), proves NOTHING changes right up to the timeout (still fully stuck just under it,
 * mirroring the "never absorbed" guarantee the normal path already has), and proves the wheel
 * un-sticks immediately once the timeout is crossed, with the gyro reading quiet (rig.gz stays
 * 0) throughout -- exactly what a slow constant bias looks like moment to moment.
 */
static void test_yaw_baseline_self_corrects_drift_even_while_scroll_latched_active() {
    Rig rig;
    rig.boot(); // seeded at yaw=0, level, gyro quiet

    // Drift has already pushed yaw past the engage threshold (10 deg): 60 deg -> 95 report
    // units, latching the scroll wheel active.
    rig.yaw = 60.0f;
    rig.step();
    TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isScrollWheelActive(), "drifted yaw should have engaged the scroll wheel");

    // Comfortably under the recovery timeout (30 s): still fully stuck, baseline untouched --
    // same "never absorbed" guarantee as a real sustained scroll. Stops at 25 s, not right at
    // the 30 s edge, to leave headroom for float accumulation error in the running dt sum (the
    // exact crossing step is not the point under test -- that it hasn't moved AT ALL well
    // before the timeout is).
    for (int i = 0; i < 1249; ++i) rig.step(); // ~25 s more
    TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isScrollWheelActive(), "must still be stuck well under the recovery timeout");
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.yawBaseline());

    // Cross the timeout (another 12 s, well past the 30 s mark with margin): the snap fires,
    // and the Schmitt trigger disengages (calibratedYaw ~0) on the step after that.
    for (int i = 0; i < 600; ++i) rig.step();

    TEST_ASSERT_FALSE_MESSAGE(rig.mouse.isScrollWheelActive(),
                               "yaw baseline must snap-correct the drift and release the scroll latch");
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, rig.calibration.yawBaseline());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rig.detector.getCalibratedYaw());
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
    rig.boot(); // seeding now averages SETTLING_SAMPLES plausible samples (#1181)
    TEST_ASSERT_TRUE(rig.calibration.isSeeded());
    // WITHIN, not EQUAL: averaging 500 copies of a non-exact-binary float (#1181) accumulates
    // a little float-summation rounding that a single-sample seed never had to deal with.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, HW_REST_X, rig.calibration.tiltBaselineX());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, HW_REST_Y, rig.calibration.tiltBaselineY());
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(rig.sink.events.size()));
}


// ---------------- Gyro-Z bias drift vs the scroll Schmitt trigger (#1181) ----------------

/**
 * Models the real driver yaw: Qmi8658C integrates raw gyro Z with NO bias removal, so the yaw
 * the detector sees ramps at the gyro's zero-rate offset even at rest (live board: gyroZ =
 * 0.5-0.6 dps at rest, calibratedYaw parked at a constant +2.62 deg in the VERIFY log). Unlike
 * the rest of the suite, which sets rig.yaw directly with gz = 0, here every step integrates
 * (realRate + bias) into rig.yaw and reports it as gz, exactly as the hardware does.
 */
struct DriftingBoard {
    Rig &rig;
    float bias;
    float trueYaw{0}; ///< Real board rotation about Z, degrees.

    void step(const float realRateDps) {
        const float measured = realRateDps + bias;
        rig.gz = measured;
        rig.yaw += measured * (static_cast<float>(Rig::PERIOD_MS) / 1000.0f);
        trueYaw += realRateDps * (static_cast<float>(Rig::PERIOD_MS) / 1000.0f);
        rig.step();
    }

    void hold(const uint32_t ms) {
        for (uint32_t t = 0; t < ms; t += Rig::PERIOD_MS) step(0.0f);
    }

    /** Hand twist to an absolute real angle at roughly `dps`. */
    void twistTo(const float targetDeg, const float dps = 60.0f) {
        const float delta = targetDeg - trueYaw;
        const float perStep = dps * (static_cast<float>(Rig::PERIOD_MS) / 1000.0f);
        const int n = static_cast<int>(std::ceil(std::abs(delta) / perStep));
        if (n == 0) return;
        const float rate = delta / (static_cast<float>(n) * static_cast<float>(Rig::PERIOD_MS) / 1000.0f);
        for (int i = 0; i < n; ++i) step(rate);
        trueYaw = targetDeg; // kill float residue
    }

    /** Power on at rest: warm-up + settling, with the yaw drifting the whole time. */
    void boot() {
        for (int i = 0; i < Calibration::DEFAULT_WARMUP_SAMPLES + Calibration::SETTLING_SAMPLES + 1; ++i) step(0.0f);
        TEST_ASSERT_TRUE(rig.calibration.isSeeded());
    }
};

/**
 * Edgar (#1181): "It should be based on the absolute angle... comes on at abs(10+), turns off at
 * abs(5)... I can't repeatably turn it on/off as I rotate around the Z axis." Repeated,
 * identical engage/hold/disengage cycles in BOTH directions with short idle pauses between,
 * on a board with the real measured gyro-Z bias. Before the fix: calibratedYaw idles at
 * bias * tau (~2.75 deg), so an 8.5 deg twist engages in the drift direction, and the frozen
 * baseline during each 10 s scroll adds ~5.5 deg more, so returning to 2 deg doesn't turn it off.
 */
static void run_repeatable_scroll_cycles_with_gyro_bias(const float bias) {
    Rig rig;
    DriftingBoard board{rig, bias};
    board.boot();
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.3f, 0.0f, rig.detector.getCalibratedYaw(),
                                     "seeded yaw must start at zero, not bias * (settling window / 2) behind");
    board.hold(30000);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.3f, 0.0f, rig.detector.getCalibratedYaw(),
                                     "a resting board must read ~0 yaw, not a standing bias * tau offset");

    for (int cycle = 0; cycle < 6; ++cycle) {
        const float dir = (cycle % 2 == 0) ? 1.0f : -1.0f;
        char msg[96];

        board.twistTo(dir * 8.5f);
        board.hold(500);
        snprintf(msg, sizeof msg, "cycle %d dir %+.0f: 8.5 deg must NOT engage", cycle, dir);
        TEST_ASSERT_FALSE_MESSAGE(rig.mouse.isScrollWheelActive(), msg);

        board.twistTo(dir * 14.0f);
        board.hold(500);
        snprintf(msg, sizeof msg, "cycle %d dir %+.0f: 14 deg must engage", cycle, dir);
        TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isScrollWheelActive(), msg);
        board.hold(10000); // a real 10 s scroll

        board.twistTo(dir * 8.0f);
        board.hold(500);
        snprintf(msg, sizeof msg, "cycle %d dir %+.0f: 8 deg (deadzone) must stay engaged", cycle, dir);
        TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isScrollWheelActive(), msg);

        board.twistTo(dir * 2.0f);
        board.hold(500);
        snprintf(msg, sizeof msg, "cycle %d dir %+.0f: 2 deg must disengage", cycle, dir);
        TEST_ASSERT_FALSE_MESSAGE(rig.mouse.isScrollWheelActive(), msg);

        board.twistTo(0.0f);
        board.hold(2000); // short idle pause before the next attempt
    }
}

static void test_scroll_toggle_is_repeatable_with_positive_gyro_bias() {
    run_repeatable_scroll_cycles_with_gyro_bias(0.55f); // the live board's measured offset
}

static void test_scroll_toggle_is_repeatable_with_negative_gyro_bias() {
    run_repeatable_scroll_cycles_with_gyro_bias(-0.55f);
}

/**
 * A long engaged scroll must not accumulate the gyro bias (baseline is frozen while scrolling).
 * Held 20 s (was 30 s): the bias (0.55 dps) is below the stuck-scroll "gyro quiet" gate, so a
 * hold at/over GestureDetector::STUCK_SCROLL_RECOVERY_S (now 30 s, #1181) is by design snapped
 * by the recovery timeout -- that is a different behaviour, not what this test is about. 20 s
 * matches test_sustained_scroll_is_not_absorbed and would still show ~11 deg of accumulated
 * bias if the baseline were not frozen, far outside the 0.3 deg tolerance below.
 */
static void test_long_scroll_hold_does_not_accumulate_gyro_bias() {
    Rig rig;
    DriftingBoard board{rig, 0.55f};
    board.boot();
    board.twistTo(-12.0f);
    board.hold(20000);
    TEST_ASSERT_TRUE(rig.mouse.isScrollWheelActive());
    TEST_ASSERT_FLOAT_WITHIN(0.3f, -12.0f, rig.detector.getCalibratedYaw());
    board.twistTo(0.0f);
    board.hold(200);
    TEST_ASSERT_FALSE(rig.mouse.isScrollWheelActive());
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, rig.detector.getCalibratedYaw());
}

/** The bias estimate follows a slow (thermal) change of the zero-rate offset while idle. */
static void test_gyro_bias_estimate_tracks_slow_change_while_idle() {
    Rig rig;
    DriftingBoard board{rig, 0.55f};
    board.boot();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.55f, rig.calibration.yawRateBiasDps());
    board.bias = 0.9f;
    board.hold(180000);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.9f, rig.calibration.yawRateBiasDps());
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, rig.detector.getCalibratedYaw());
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0); // keep Unity output even if a sanitizer aborts the run
    UNITY_BEGIN();
    RUN_TEST(test_seeds_both_baselines_after_warmup);
    RUN_TEST(test_adapts_only_when_idle);
    RUN_TEST(test_tilt_idle_and_yaw_idle_flags_are_independently_honoured);
    RUN_TEST(test_time_constant_is_slow_and_exponential);
    RUN_TEST(test_default_yaw_ema_converges_within_10_seconds_while_idle);
    RUN_TEST(test_new_heading_becomes_zero_within_10_seconds_through_detector);
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
    RUN_TEST(test_settling_average_includes_varying_yaw_not_just_tilt);
    RUN_TEST(test_plausible_resting_sample_window);
    RUN_TEST(test_garbage_startup_samples_then_real_rest_produces_no_phantom_movement);
    RUN_TEST(test_yaw_baseline_self_corrects_drift_even_while_scroll_latched_active);
    RUN_TEST(test_rezero_during_warmup_does_not_seed_from_garbage);
    RUN_TEST(test_seeding_while_still_being_handled_is_rejected_until_genuinely_still);
    RUN_TEST(test_scroll_toggle_is_repeatable_with_positive_gyro_bias);
    RUN_TEST(test_scroll_toggle_is_repeatable_with_negative_gyro_bias);
    RUN_TEST(test_long_scroll_hold_does_not_accumulate_gyro_bias);
    RUN_TEST(test_gyro_bias_estimate_tracks_slow_change_while_idle);
    return UNITY_END();
}
