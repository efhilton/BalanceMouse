// The gesture -> event-sink seam: simulated IMU input in, discrete events out, via a fake
// sink (no hardware, no BLE). Also covers HID report encoding and the fan-out.
#include "test_support.hpp"

#include <FeedbackState.hpp>
#include <MouseHidReport.hpp>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unity.h>

using namespace Hilton;
using namespace TestSupport;
using Type = RecordedEvent::Type;

void setUp() {
}

void tearDown() {
}

static void assertEvent(const RecordedEvent &e, const Type type, const Button button) {
    TEST_ASSERT_TRUE(e.type == type);
    TEST_ASSERT_TRUE(e.button == button);
}

static void test_left_click_emits_down_then_up() {
    Rig rig;
    rig.boot();
    rig.spike(-150.0f, 0.0f); // fast roll left
    rig.run(100);
    rig.spike(+150.0f, 0.0f); // fast roll back
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(rig.sink.events.size()));
    assertEvent(rig.sink.events[0], Type::Down, Button::Left);
    assertEvent(rig.sink.events[1], Type::Up, Button::Left);
}

static void test_right_click_emits_down_then_up() {
    Rig rig;
    rig.boot();
    rig.spike(+150.0f, 0.0f);
    rig.run(100);
    rig.spike(-150.0f, 0.0f);
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(rig.sink.events.size()));
    assertEvent(rig.sink.events[0], Type::Down, Button::Right);
    assertEvent(rig.sink.events[1], Type::Up, Button::Right);
}

static void test_middle_click_emits_down_then_up() {
    Rig rig;
    rig.boot();
    rig.spike(0.0f, -150.0f); // pitch forward
    rig.run(100);
    rig.spike(0.0f, +150.0f);
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(rig.sink.events.size()));
    assertEvent(rig.sink.events[0], Type::Down, Button::Middle);
    assertEvent(rig.sink.events[1], Type::Up, Button::Middle);
}

static void test_gyro_below_threshold_is_not_a_click() {
    Rig rig;
    rig.boot();
    rig.spike(-99.0f, 0.0f);
    rig.spike(0.0f, -100.0f); // threshold is strictly greater than 100
    rig.spike(0.0f, +150.0f); // pitch BACK never presses Middle
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(rig.sink.events.size()));
}

static void test_slow_return_keeps_button_held_for_drag() {
    Rig rig;
    rig.boot();
    rig.spike(-150.0f, 0.0f);
    rig.gx = 40.0f; // slow roll back: below threshold, must NOT release
    rig.run(2000);
    TEST_ASSERT_EQUAL_INT(1, rig.sink.count(Type::Down, Button::Left));
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Up));
    TEST_ASSERT_TRUE(rig.mouse.isButtonPressed(Button::Left));
}

static void test_release_lockout_blocks_immediate_re_press() {
    Rig rig;
    rig.boot();
    rig.spike(-150.0f, 0.0f);
    rig.run(100);
    rig.spike(+150.0f, 0.0f); // release at t0
    rig.run(200); // ~220 ms after release
    rig.spike(-150.0f, 0.0f); // within 300 ms: ignored
    TEST_ASSERT_EQUAL_INT(1, rig.sink.count(Type::Down));
    rig.run(200);
    rig.spike(-150.0f, 0.0f); // > 300 ms: accepted
    TEST_ASSERT_EQUAL_INT(2, rig.sink.count(Type::Down));
}

static void test_no_on_device_double_click() {
    // Two quick clicks are reported as two independent down/up pairs -- nothing synthesised.
    Rig rig;
    rig.boot();
    for (int i = 0; i < 2; ++i) {
        rig.spike(-150.0f, 0.0f);
        rig.run(60);
        rig.spike(+150.0f, 0.0f);
        rig.run(320);
    }
    TEST_ASSERT_EQUAL_INT(4, static_cast<int>(rig.sink.events.size()));
    assertEvent(rig.sink.events[0], Type::Down, Button::Left);
    assertEvent(rig.sink.events[1], Type::Up, Button::Left);
    assertEvent(rig.sink.events[2], Type::Down, Button::Left);
    assertEvent(rig.sink.events[3], Type::Up, Button::Left);
}

static void test_movement_deltas_follow_calibrated_tilt_with_hysteresis() {
    Rig rig;
    rig.boot();

    // Curves to 5 units: below the cursor start (6); 11.7 on the linear idle gauge, below
    // TILT_IDLE_MAGNITUDE_THRESHOLD (13), so idle.
    rig.ax = 0.9f;
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Move));

    // The sub-threshold lean above was idle, so the tilt baseline crept toward 0.9 a little
    // (that is the slow calibration doing its job); deltas are relative to that baseline,
    // which is frozen from here on because the board is moving.
    const float baseX = rig.calibration.tiltBaselineX();
    TEST_ASSERT_TRUE(baseX > 0.0f && baseX < 0.1f);
    // Deltas now go through the quadratic acceleration curve (#1181), full speed at
    // MOVEMENT_FULL_SCALE_TILT (3.3552 m/s^2, 20 deg), output scaled by MOVEMENT_SPEED_SCALE (0.5).
    rig.ax = 3.0f; // n ~ 0.88 -> ~49 units
    rig.ay = -1.5f; // n = 0.447 -> 0.1999 * 63.5 = 12.7 -> 13; Y is negated -> HID dy = +13
    rig.sink.clear();
    rig.run(200); // 10 samples
    TEST_ASSERT_EQUAL_INT(10, rig.sink.count(Type::Move));
    const int expectedDx = GestureDetector::movementDeltaFromTilt(3.0f - baseX);
    TEST_ASSERT_INT_WITHIN(2, 49, expectedDx); // baseX in (0, 0.1) -> 47..51
    for (const auto &e: rig.sink.events) {
        TEST_ASSERT_EQUAL_INT(expectedDx, e.dx);
        // Y is negated on the HID path (GestureDetector::handleMouseMovement, #1181):
        // calibratedY < 0 -> dy > 0.
        TEST_ASSERT_EQUAL_INT(+13, e.dy);
    }
    TEST_ASSERT_EQUAL_FLOAT(baseX, rig.calibration.tiltBaselineX());

    // Between stop (2) and start (6) -> keeps moving. Under the half-speed curve 0.8 m/s^2
    // gives n ~ 0.21..0.24 -> 3..4 units.
    rig.ax = 0.8f;
    rig.ay = 0.0f;
    rig.sink.clear();
    rig.run(100);
    TEST_ASSERT_EQUAL_INT(5, rig.sink.count(Type::Move));
    const int expectedMidDx = GestureDetector::movementDeltaFromTilt(0.8f - baseX);
    TEST_ASSERT_TRUE(expectedMidDx > Mouse::MOVEMENT_THRESHOLD_STOP && expectedMidDx < Mouse::MOVEMENT_THRESHOLD_START);
    TEST_ASSERT_EQUAL_INT(expectedMidDx, rig.sink.events.back().dx);

    rig.ax = 0.2f; // curves to 0 units: below stop
    rig.sink.clear();
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Move));
    TEST_ASSERT_FALSE(rig.mouse.isMoving());
}

/**
 * HID movement acceleration curve (#1181, Edgar: "the further we are away from the center, the
 * more 'acceleration' the mouse should see"). Pins the default quadratic curve exactly, at the
 * halved output scale (MOVEMENT_SPEED_SCALE = 0.5, Edgar: "It's crazy fast"; full speed is now
 * round(127 * 0.5) = 64). The exponent/full-scale/speed scale are feel-tuning values -- if they
 * are retuned, update the expected numbers here deliberately.
 */
static void test_movement_acceleration_curve_shape() {
    using G = GestureDetector;
    constexpr float FULL = G::MOVEMENT_FULL_SCALE_TILT;
    TEST_ASSERT_EQUAL_FLOAT(2.0f, G::MOVEMENT_ACCEL_EXPONENT);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, G::MOVEMENT_SPEED_SCALE);

    // Centre and saturation.
    TEST_ASSERT_EQUAL_INT(0, G::movementDeltaFromTilt(0.0f));
    TEST_ASSERT_EQUAL_INT(0, G::movementDeltaFromTilt(-0.0f));
    TEST_ASSERT_EQUAL_INT(0, G::movementDeltaFromTilt(std::numeric_limits<float>::quiet_NaN()));
    TEST_ASSERT_EQUAL_INT(64, G::movementDeltaFromTilt(FULL));
    TEST_ASSERT_EQUAL_INT(-64, G::movementDeltaFromTilt(-FULL));
    TEST_ASSERT_EQUAL_INT(64, G::movementDeltaFromTilt(2.0f * FULL));
    TEST_ASSERT_EQUAL_INT(64, G::movementDeltaFromTilt(9.81f));
    TEST_ASSERT_EQUAL_INT(-64, G::movementDeltaFromTilt(-1.0e9f));
    TEST_ASSERT_EQUAL_INT(64, G::movementDeltaFromTilt(std::numeric_limits<float>::infinity()));
    TEST_ASSERT_EQUAL_INT(-64, G::movementDeltaFromTilt(-std::numeric_limits<float>::infinity()));

    // Half of full tilt -> a QUARTER of full speed (quadratic): 0.25 * 63.5 = 15.875 -> 16
    // (exactly 64 / 4).
    TEST_ASSERT_EQUAL_INT(16, G::movementDeltaFromTilt(0.5f * FULL));
    TEST_ASSERT_EQUAL_INT(-16, G::movementDeltaFromTilt(-0.5f * FULL));
    TEST_ASSERT_EQUAL_INT(G::movementDeltaFromTilt(FULL), 4 * G::movementDeltaFromTilt(0.5f * FULL));
    // Quarter tilt -> 1/16: 3.97 -> 4. Three-quarter tilt -> 9/16: 35.7 -> 36.
    TEST_ASSERT_EQUAL_INT(4, G::movementDeltaFromTilt(0.25f * FULL));
    TEST_ASSERT_EQUAL_INT(36, G::movementDeltaFromTilt(0.75f * FULL));

    // Fine control near centre: a small tilt gives disproportionately LESS than the old linear
    // mapping (GRAVITY_SCALE * tilt, truncated). 0.5 m/s^2 (~2.9 deg): old 6, now 1.
    const int oldSmall = static_cast<int>(G::GRAVITY_SCALE * 0.5f);
    const int newSmall = G::movementDeltaFromTilt(0.5f);
    TEST_ASSERT_EQUAL_INT(6, oldSmall);
    TEST_ASSERT_EQUAL_INT(1, newSmall);
    TEST_ASSERT_LESS_THAN_INT(oldSmall, newSmall);
    // ...and still MORE than the old linear mapping out near full tilt, even at half speed:
    // 3.0 m/s^2 (~17.8 deg): old 38, now 51.
    TEST_ASSERT_EQUAL_INT(38, static_cast<int>(G::GRAVITY_SCALE * 3.0f));
    TEST_ASSERT_EQUAL_INT(51, G::movementDeltaFromTilt(3.0f));
    TEST_ASSERT_GREATER_THAN_INT(38, G::movementDeltaFromTilt(3.0f));

    // Faster than linear: doubling the tilt (below full scale) roughly quadruples the output,
    // and the curve is monotonic and exactly odd-symmetric (sign handled separately). Uses
    // 0.4 -> 0.8 (10 -> 41) rather than 0.2 -> 0.4: at the halved scale 0.2 tilt is only 2.54 ->
    // 3, and x4 amplifies that rounding error past +/-1.
    int previous = 0;
    for (int i = 1; i <= 120; ++i) {
        const float tilt = FULL * static_cast<float>(i) / 100.0f;
        const int pos = G::movementDeltaFromTilt(tilt);
        TEST_ASSERT_TRUE(pos >= previous);
        TEST_ASSERT_TRUE(pos >= 0);
        TEST_ASSERT_EQUAL_INT(-pos, G::movementDeltaFromTilt(-tilt));
        previous = pos;
    }
    TEST_ASSERT_INT_WITHIN(1, 4 * G::movementDeltaFromTilt(0.4f * FULL), G::movementDeltaFromTilt(0.8f * FULL));
}

/** The curve and the sign mapping end to end, on BOTH axes (X passes through, Y is negated). */
static void test_movement_curve_preserves_sign_on_both_axes() {
    const float half = 0.5f * GestureDetector::MOVEMENT_FULL_SCALE_TILT;
    const float small = 0.2f * GestureDetector::MOVEMENT_FULL_SCALE_TILT; // 0.04 * 63.5 = 2.54 -> 3
    struct Case {
        float ax, ay;
        int dx, dy;
    };
    const Case cases[] = {
        {+half, 0, +16, 0}, // tilt right   -> cursor right
        {-half, 0, -16, 0}, // tilt left    -> cursor left
        {0, +half, 0, -16}, // calibratedY > 0 -> dy < 0 (HID Y negated, #1181)
        {0, -half, 0, +16}, // calibratedY < 0 -> dy > 0
        {+half, -half, +16, +16}, // diagonal: both curves independent, signs independent
        {-half, +half, -16, -16},
    };
    for (const auto &c: cases) {
        Rig rig;
        rig.boot();
        TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineX());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.calibration.tiltBaselineY());
        rig.ax = c.ax;
        rig.ay = c.ay;
        rig.sink.clear();
        rig.run(100);
        TEST_ASSERT_EQUAL_INT(5, rig.sink.count(Type::Move));
        for (const auto &e: rig.sink.events) {
            TEST_ASSERT_EQUAL_INT(c.dx, e.dx);
            TEST_ASSERT_EQUAL_INT(c.dy, e.dy);
        }
    }
    // A small per-axis tilt curves to 3 per axis: hypot(3, 3) = 4.24 < start (6), so a small
    // diagonal lean does not start movement -- fine control near centre.
    TEST_ASSERT_EQUAL_INT(3, GestureDetector::movementDeltaFromTilt(small));
    TEST_ASSERT_EQUAL_INT(-3, GestureDetector::movementDeltaFromTilt(-small));
}

/**
 * MOVEMENT_FULL_SCALE_TILT (gestures) intentionally duplicates FeedbackRenderer's
 * BALL_FULL_SCALE_TILT (led_feedback, which does not depend on gestures) so the cursor reaches
 * full speed at the same tilt that puts the LED ball at the matrix edge. Fails if they drift
 * apart, or if the m/s^2 value no longer matches its degree spec.
 */
static void test_movement_full_scale_matches_ball_full_scale() {
    constexpr float PI = 3.14159265358979f;
    TEST_ASSERT_EQUAL_FLOAT(FeedbackRenderer::BALL_FULL_SCALE_TILT_DEG, GestureDetector::MOVEMENT_FULL_SCALE_TILT_DEG);
    TEST_ASSERT_EQUAL_FLOAT(FeedbackRenderer::BALL_FULL_SCALE_TILT, GestureDetector::MOVEMENT_FULL_SCALE_TILT);
    const float expected = Calibration::STANDARD_GRAVITY_MS2
                           * std::sin(GestureDetector::MOVEMENT_FULL_SCALE_TILT_DEG * PI / 180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, GestureDetector::MOVEMENT_FULL_SCALE_TILT);
}

static void test_scroll_ticks_have_direction_and_rate() {
    Rig rig;
    rig.boot();
    rig.yaw = 30.0f; // 47 units (SCROLL_SCALE = 127/80, #1181) -> ~14.1 ticks/s
    rig.run(1000);
    TEST_ASSERT_INT_WITHIN(1, 14, rig.sink.scrollTotal());
    for (const auto &e: rig.sink.events) TEST_ASSERT_TRUE(e.type == Type::Scroll && e.ticks > 0);

    rig.yaw = 0.0f; // back to centre: stops
    rig.run(200);
    rig.sink.clear();
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.scrollTotal());

    rig.yaw = -30.0f;
    rig.run(1000);
    TEST_ASSERT_INT_WITHIN(1, -14, rig.sink.scrollTotal());

    // Below the engage threshold (~10 deg, #1181, Edgar's Schmitt-trigger spec): no scrolling.
    Rig quiet;
    quiet.boot();
    quiet.yaw = 7.0f;
    quiet.run(2000);
    TEST_ASSERT_EQUAL_INT(0, quiet.sink.count(Type::Scroll));
}

/**
 * The Schmitt trigger itself (#1181, Edgar: "engage going up stays off until abs(angle) hits
 * 10, but once engaged, don't disengage again until abs(angle) drops back down to about 5" --
 * a real deadzone, not a single threshold). Exercises exactly the gap a single threshold would
 * get wrong: a value that would never re-trigger a naive ">10" check but also must not be
 * treated as "stopped" once already engaged.
 */
static void test_scroll_engage_disengage_has_a_real_deadzone() {
    Rig rig;
    rig.boot();

    rig.yaw = 10.0f; // AT the engage threshold, not past it: must not engage yet
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Scroll));
    TEST_ASSERT_FALSE(rig.mouse.isScrollWheelActive());

    rig.yaw = 11.0f; // past 10: engages
    rig.run(500);
    TEST_ASSERT_TRUE(rig.mouse.isScrollWheelActive());
    TEST_ASSERT_GREATER_THAN_INT(0, rig.sink.count(Type::Scroll));

    // The deadzone itself: 7 deg is below the 10-deg engage point (never engages a resting
    // board from here) but ABOVE the ~5-deg disengage point -- once already engaged, it must
    // stay engaged, not chatter off because it dropped below 10.
    rig.yaw = 7.0f;
    rig.run(500);
    TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isScrollWheelActive(), "must stay engaged inside the deadzone");

    rig.yaw = 4.0f; // below ~5: now it disengages
    rig.run(500);
    TEST_ASSERT_FALSE(rig.mouse.isScrollWheelActive());

    // And confirm 7 deg alone, from a cold/disengaged start, never engages it either.
    Rig rig2;
    rig2.boot();
    rig2.yaw = 7.0f;
    rig2.run(2000);
    TEST_ASSERT_FALSE(rig2.mouse.isScrollWheelActive());
}

/**
 * GestureDetector::SCROLL_ENGAGE_DEG/SCROLL_DISENGAGE_DEG document the intended degree
 * thresholds; Mouse::ANGLE_THRESHOLD_START/STOP are the actual report-unit enforcement (Mouse
 * has no notion of degrees). This guards them from drifting apart -- e.g. if
 * MAX_SCROLL_ANGLE_DEG is retuned later ("tunable later", #1181, Edgar) without recomputing
 * Mouse's thresholds to match.
 */
static void test_scroll_engage_disengage_thresholds_match_mouse_hysteresis() {
    // Mouse engages once its truncated report angle EXCEEDS ANGLE_THRESHOLD_START, i.e. first
    // becomes (START + 1); it disengages once the report angle drops below STOP, i.e. last
    // holds (STOP - 1). Those integer crossings, converted back to degrees, are the true
    // behavioural thresholds -- not START/STOP divided by the scale directly, which land up to
    // one report-unit (~0.63 deg at MAX_SCROLL_ANGLE_DEG = 80) short/over.
    const float engageDeg = static_cast<float>(Mouse::ANGLE_THRESHOLD_START + 1) / GestureDetector::SCROLL_SCALE;
    const float disengageDeg = static_cast<float>(Mouse::ANGLE_THRESHOLD_STOP) / GestureDetector::SCROLL_SCALE;
    TEST_ASSERT_FLOAT_WITHIN(0.7f, GestureDetector::SCROLL_ENGAGE_DEG, engageDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.7f, GestureDetector::SCROLL_DISENGAGE_DEG, disengageDeg);
}

/**
 * Movement counterpart of the scroll guard above (#1181, Edgar: "the deadzone for xy movement
 * is very large"). Mouse::MOVEMENT_THRESHOLD_START/STOP are compared against the ALREADY-curved
 * and scaled HID deltas, so retuning MOVEMENT_SPEED_SCALE / MOVEMENT_ACCEL_EXPONENT /
 * MOVEMENT_FULL_SCALE_TILT(_DEG) silently moves the deadzone angle -- exactly what happened when
 * the 0.5 speed scale went in without rescaling 13/5 (start ~6.4 -> ~9 deg). This sweeps the real
 * curve, finds the tilt angles at which the hysteresis is actually crossed (single axis, so
 * spd == |dx|), and pins them to the intended ~6.1 deg start / ~3.5 deg stop within 1 deg.
 * The old 13/5 at the 0.5 scale cross at ~9.1 / ~5.2 deg and would fail this.
 */
static void test_movement_start_stop_thresholds_match_intended_deadzone() {
    constexpr float INTENDED_START_DEG = 6.1f;
    constexpr float INTENDED_STOP_DEG = 3.5f;
    constexpr float TOLERANCE_DEG = 1.0f;
    constexpr float STEP_DEG = 0.01f;
    const float degToRad = std::acos(-1.0f) / 180.0f;
    auto deltaAtDeg = [degToRad](const float deg) {
        return static_cast<int>(GestureDetector::movementDeltaFromTilt(
            Calibration::STANDARD_GRAVITY_MS2 * std::sin(deg * degToRad)));
    };

    // Start: Mouse starts moving once spd > START -> first angle where |dx| > START.
    float startDeg = -1.0f;
    // Stop: Mouse stops once spd < STOP -> below the first angle where |dx| >= STOP.
    float stopDeg = -1.0f;
    for (float deg = 0.0f; deg <= GestureDetector::MOVEMENT_FULL_SCALE_TILT_DEG; deg += STEP_DEG) {
        const int d = deltaAtDeg(deg);
        if (stopDeg < 0.0f && d >= Mouse::MOVEMENT_THRESHOLD_STOP) {
            stopDeg = deg;
        }
        if (startDeg < 0.0f && d > Mouse::MOVEMENT_THRESHOLD_START) {
            startDeg = deg;
            break;
        }
    }
    printf("movement deadzone: start at %.2f deg, stop below %.2f deg\n", startDeg, stopDeg);
    TEST_ASSERT_TRUE_MESSAGE(startDeg > 0.0f, "start threshold never crossed below full-scale tilt");
    TEST_ASSERT_TRUE_MESSAGE(stopDeg > 0.0f, "stop threshold never crossed below full-scale tilt");
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE_DEG, INTENDED_START_DEG, startDeg);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE_DEG, INTENDED_STOP_DEG, stopDeg);
    // And it must still be a real hysteresis gap, not a single threshold.
    TEST_ASSERT_TRUE(startDeg - stopDeg > 1.0f);
}

static void test_scroll_suppressed_while_button_held_and_buttons_suppressed_while_scrolling() {
    Rig rig;
    rig.boot();
    rig.spike(-150.0f, 0.0f); // left down
    rig.yaw = 30.0f;
    rig.run(1000);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Scroll));

    Rig rig2;
    rig2.boot();
    rig2.yaw = 30.0f;
    rig2.run(200);
    rig2.spike(-150.0f, 0.0f); // ignored: scroll has priority
    rig2.run(200);
    TEST_ASSERT_EQUAL_INT(0, rig2.sink.count(Type::Down));
    TEST_ASSERT_GREATER_THAN_INT(0, rig2.sink.count(Type::Scroll));
}

/**
 * Scroll vs. incidental side tilt (#1181, 2026-11-02). Twisting the board around Z to scroll also
 * tips it sideways 10-15 deg. In live ON-state captures the side tilt had a median of 24.6 on the
 * linear GRAVITY_SCALE metric (90th percentile 38.7), and in 87% of samples it was above the old
 * movement start (13). That tilt latched mouse.isMoving() before yaw crossed SCROLL_ENGAGE_DEG,
 * and handleScrollWheel's old `|| mouse.isMoving()` bail-out then blocked scroll for good. See the
 * doc comment on GestureDetector::handleScrollWheel.
 *
 * Tilt units: rig.ax is the accelerometer X in m/s^2 (Rig boots with the baseline at 0).
 * 1.9 m/s^2 = 9.8 * sin(~11.2 deg) -> 127/9.8 * 1.9 = 24.6 linear, the captured median.
 */
static constexpr float CAPTURED_MEDIAN_SIDE_TILT{1.9f}; // ~11.2 deg, 24.6 linear
static constexpr float CAPTURED_P90_SIDE_TILT{2.99f}; // ~17.8 deg, 38.7 linear

static void test_scroll_engages_despite_accompanying_side_tilt() {
    Rig rig;
    rig.boot();
    TEST_ASSERT_INT_WITHIN(1, 25, static_cast<int>(GestureDetector::GRAVITY_SCALE * CAPTURED_MEDIAN_SIDE_TILT));

    // Real ordering: the side tilt arrives first and latches the movement hysteresis...
    rig.ax = CAPTURED_MEDIAN_SIDE_TILT;
    rig.run(200);
    TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isMoving(), "precondition: the side tilt alone latches movement");
    TEST_ASSERT_GREATER_THAN_INT(0, rig.sink.count(Type::Move));

    // ...then yaw crosses SCROLL_ENGAGE_DEG with the tilt still held. Scroll must engage.
    rig.yaw = GestureDetector::SCROLL_ENGAGE_DEG + 5.0f;
    rig.sink.clear();
    rig.run(1000);
    TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isScrollWheelActive(),
                             "scroll must engage even though the cursor was already moving");
    TEST_ASSERT_GREATER_THAN_INT(0, rig.sink.scrollTotal());
    // The stale movement latch is released while scrolling, so the stuck-scroll recovery and
    // the yaw idle gate (both of which require !isMoving()) are not blocked.
    TEST_ASSERT_FALSE(rig.mouse.isMoving());
}

/**
 * Unchanged by the fix: while scroll is engaged, movement output stays fully suppressed at the
 * CURRENT thresholds (start 6 / stop 2), for any tilt, including near full scale. Covers both
 * orderings (scroll first then tilt, and tilt first then scroll), then checks that movement
 * resumes once scroll lets go.
 */
static void test_active_scroll_fully_suppresses_movement() {
    TEST_ASSERT_EQUAL_INT(6, Mouse::MOVEMENT_THRESHOLD_START);
    TEST_ASSERT_EQUAL_INT(2, Mouse::MOVEMENT_THRESHOLD_STOP);

    // Ordering A: engage scroll from a level board, then tilt hard.
    Rig a;
    a.boot();
    a.yaw = 20.0f;
    a.run(200);
    TEST_ASSERT_TRUE(a.mouse.isScrollWheelActive());
    a.sink.clear();
    a.ax = CAPTURED_P90_SIDE_TILT;
    a.ay = -2.0f;
    a.run(1000);
    TEST_ASSERT_EQUAL_INT(0, a.sink.count(Type::Move));
    TEST_ASSERT_GREATER_THAN_INT(0, a.sink.scrollTotal());
    TEST_ASSERT_FALSE(a.mouse.isMoving());
    TEST_ASSERT_EQUAL_INT(0, a.mouse.getMovement().deltaX);
    TEST_ASSERT_EQUAL_INT(0, a.mouse.getMovement().deltaY);

    // Ordering B: moving first, then scroll engages. Not a single Move after the engage sample.
    Rig b;
    b.boot();
    b.ax = CAPTURED_MEDIAN_SIDE_TILT;
    b.run(200);
    TEST_ASSERT_TRUE(b.mouse.isMoving());
    b.yaw = 20.0f;
    b.step(); // engages on this sample (scroll is dispatched first)
    TEST_ASSERT_TRUE(b.mouse.isScrollWheelActive());
    b.sink.clear();
    b.ax = CAPTURED_P90_SIDE_TILT;
    b.run(1000);
    TEST_ASSERT_EQUAL_INT(0, b.sink.count(Type::Move));
    TEST_ASSERT_GREATER_THAN_INT(0, b.sink.scrollTotal());

    // Scroll released (yaw back to centre), tilt still held: movement resumes from rest.
    b.yaw = 0.0f;
    b.sink.clear();
    b.run(400);
    TEST_ASSERT_FALSE(b.mouse.isScrollWheelActive());
    TEST_ASSERT_GREATER_THAN_INT(0, b.sink.count(Type::Move));
}

/**
 * Corner case, asserted explicitly: a side tilt that would have counted as "moving" even under
 * the OLD, larger 13/5 movement thresholds still lets scroll engage. Uses the captured 90th
 * percentile (38.7 linear). That is above 13 on the linear metric AND on the curved HID delta
 * that Mouse's hysteresis actually compares against, so it is "moving" under any threshold this
 * project has used.
 */
static void test_scroll_engages_with_tilt_that_moved_under_old_thresholds() {
    constexpr int OLD_MOVEMENT_THRESHOLD_START = 13;
    TEST_ASSERT_GREATER_THAN_INT(OLD_MOVEMENT_THRESHOLD_START,
                                 static_cast<int>(GestureDetector::GRAVITY_SCALE * CAPTURED_P90_SIDE_TILT));
    TEST_ASSERT_GREATER_THAN_INT(OLD_MOVEMENT_THRESHOLD_START,
                                 GestureDetector::movementDeltaFromTilt(CAPTURED_P90_SIDE_TILT));

    Rig rig;
    rig.boot();
    rig.ax = CAPTURED_P90_SIDE_TILT;
    rig.run(200);
    TEST_ASSERT_TRUE(rig.mouse.isMoving());

    // Yaw just past the engage point (Mouse needs report angle > START: SCROLL_ENGAGE_DEG + 1
    // clears that comfortably), not a huge twist.
    rig.yaw = GestureDetector::SCROLL_ENGAGE_DEG + 1.0f;
    rig.sink.clear();
    rig.run(1000);
    TEST_ASSERT_TRUE_MESSAGE(rig.mouse.isScrollWheelActive(),
                             "a tilt that was 'moving' under the old 13/5 thresholds must not block scroll");
    TEST_ASSERT_GREATER_THAN_INT(0, rig.sink.scrollTotal());
    // Scroll is dispatched first, so it wins from the very first sample yaw is past the engage
    // point: no Move events at all in this window.
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Move));
}

static void test_drag_sequence_produces_down_moves_up() {
    Rig rig;
    rig.boot();
    rig.spike(-150.0f, 0.0f);
    rig.ax = 2.0f;
    rig.run(400);
    rig.ax = 0.0f;
    rig.run(400);
    rig.spike(+150.0f, 0.0f);
    TEST_ASSERT_TRUE(rig.sink.events.front().type == Type::Down);
    TEST_ASSERT_TRUE(rig.sink.events.back().type == Type::Up);
    TEST_ASSERT_GREATER_THAN_INT(5, rig.sink.count(Type::Move));
}

// ---------------- HID report encoding (what BleHidMouse transmits) ----------------

static void test_hid_report_layout() {
    const MouseHidReport r = makeMouseHidReport(HidButtonBits::LEFT | HidButtonBits::MIDDLE, -5, 127, -1);
    TEST_ASSERT_EQUAL_HEX8(0x05, r[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFB, r[1]);
    TEST_ASSERT_EQUAL_HEX8(0x7F, r[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, hidButtonBit(Button::Left));
    TEST_ASSERT_EQUAL_HEX8(0x02, hidButtonBit(Button::Right));
    TEST_ASSERT_EQUAL_HEX8(0x04, hidButtonBit(Button::Middle));
}

static void test_report_builder_one_report_per_event_with_button_mask() {
    std::vector<MouseHidReport> sent;
    MouseHidReportBuilder builder([&sent](const MouseHidReport &r) { sent.push_back(r); });

    builder.onButtonDown(Button::Left);
    builder.onMovement(10, -3);
    builder.onButtonDown(Button::Right);
    builder.onScroll(-2);
    builder.onMovement(0, 0); // no-op: nothing to report
    builder.onScroll(0); // no-op
    builder.onButtonUp(Button::Left);
    builder.onButtonUp(Button::Right);

    TEST_ASSERT_EQUAL_INT(6, static_cast<int>(sent.size()));
    TEST_ASSERT_EQUAL_HEX8(0x01, sent[0][0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, sent[1][0]); // movement carries the held button
    TEST_ASSERT_EQUAL_INT(10, static_cast<int8_t>(sent[1][1]));
    TEST_ASSERT_EQUAL_INT(-3, static_cast<int8_t>(sent[1][2]));
    TEST_ASSERT_EQUAL_HEX8(0x03, sent[2][0]);
    TEST_ASSERT_EQUAL_HEX8(0x03, sent[3][0]);
    // Ticks are yaw-signed; the HID wheel byte is negated (MouseHidReportBuilder::onScroll).
    TEST_ASSERT_EQUAL_INT(+2, static_cast<int8_t>(sent[3][3]));
    TEST_ASSERT_EQUAL_HEX8(0x02, sent[4][0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, sent[5][0]);
    for (const auto &r: sent) {
        if (r[1] == 0 && r[2] == 0 && r[3] == 0) continue;
        TEST_ASSERT_TRUE(r[3] == 0 || (r[1] == 0 && r[2] == 0)); // movement and wheel never mixed
    }
}

static void test_full_chain_gestures_to_hid_reports_via_fanout() {
    std::vector<MouseHidReport> sent;
    MouseHidReportBuilder builder([&sent](const MouseHidReport &r) { sent.push_back(r); });
    RecordingSink led; // stands in for the LED feedback sink
    MouseEventFanout fanout;
    fanout.addSink(&builder);
    fanout.addSink(&led);
    fanout.addSink(nullptr); // ignored

    Mouse mouse;
    Calibration calibration;
    GestureDetector detector(mouse, calibration, &fanout);
    uint32_t t = 0;
    auto feed = [&](const float gx, const float ax, const float yaw) {
        MotionSample s{};
        s.accel = {ax, 0, 9.8f};
        s.gyro = {gx, 0, 0};
        s.yaw = yaw;
        s.timestampMs = t;
        t += 20;
        detector.process(s);
    };
    // Warm-up + settling (#1181): seeding now averages Calibration::SETTLING_SAMPLES genuinely
    // still samples instead of taking just one.
    for (int i = 0; i < Calibration::DEFAULT_WARMUP_SAMPLES + Calibration::SETTLING_SAMPLES; ++i) feed(0, 0, 0);
    feed(-150, 0, 0); // left down
    for (int i = 0; i < 5; ++i) feed(0, 0, 0);
    feed(+150, 0, 0); // left up
    for (int i = 0; i < 20; ++i) feed(0, 0, 0);
    for (int i = 0; i < 5; ++i) feed(0, 3.0f, 0); // move right
    for (int i = 0; i < 20; ++i) feed(0, 0, 0);
    // +Z rotation: 30 deg -> 47 units -> ~14.1 ticks/s (#1181). Chevron spec: +Z = scroll DOWN,
    // so the HID wheel (+ = up) must be NEGATIVE.
    for (int i = 0; i < 50; ++i) feed(0, 0, 30.0f);

    TEST_ASSERT_EQUAL_HEX8(0x01, sent.at(0)[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, sent.at(1)[0]);
    // 3.0 m/s^2 through the quadratic curve (#1181) at half speed (MOVEMENT_SPEED_SCALE 0.5):
    // (3.0/3.3552)^2 * 127 * 0.5 = 50.8 -> 51.
    TEST_ASSERT_EQUAL_INT(51, static_cast<int8_t>(sent.at(2)[1]));
    int wheel = 0;
    for (const auto &r: sent) wheel += static_cast<int8_t>(r[3]);
    TEST_ASSERT_INT_WITHIN(1, -14, wheel);
    TEST_ASSERT_INT_WITHIN(1, 14, led.scrollTotal()); // the event stream itself stays yaw-signed
    // Every event reached both sinks.
    TEST_ASSERT_EQUAL_INT(1, led.count(Type::Down));
    TEST_ASSERT_EQUAL_INT(1, led.count(Type::Up));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(sent.size()), static_cast<int>(led.events.size()));
}

/**
 * Pins the HID sign mapping (#1181, Edgar's final real-hardware instruction). Cursor: X passes
 * straight through, Y is negated (HID only -- the LED display's own Y handling in
 * FeedbackState::updateMotion is separate and unchanged). Wheel: derived from the
 * hardware-confirmed chevron (MouseHidReportBuilder::onScroll).
 *   calibratedX > 0 -> HID dx > 0 (cursor right)
 *   calibratedX < 0 -> HID dx < 0 (cursor left)
 *   calibratedY > 0 -> HID dy < 0
 *   calibratedY < 0 -> HID dy > 0
 *   +Z rotation  (calibratedYaw > 0) -> HID wheel < 0 (scroll DOWN), matching the chevron (-1)
 *   -Z rotation  (calibratedYaw < 0) -> HID wheel > 0 (scroll UP),   matching the chevron (+1)
 */
static void test_hid_sign_mapping_matches_confirmed_display_sense() {
    struct Chain {
        std::vector<MouseHidReport> sent;
        MouseHidReportBuilder builder{[this](const MouseHidReport &r) { sent.push_back(r); }};
        uint32_t now{0};
        FeedbackState feedback{[this] { return now; }};
        MouseEventFanout fanout;
        Mouse mouse;
        Calibration calibration;
        GestureDetector detector{mouse, calibration, &fanout};
        uint32_t t{0};

        Chain() {
            fanout.addSink(&builder);
            fanout.addSink(&feedback);
            for (int i = 0; i < Calibration::DEFAULT_WARMUP_SAMPLES + Calibration::SETTLING_SAMPLES; ++i) {
                feed(0, 0, 0);
            }
            sent.clear();
        }

        void feed(const float ax, const float ay, const float yaw) {
            MotionSample s{};
            s.accel = {ax, ay, 9.8f};
            s.yaw = yaw;
            s.timestampMs = t;
            t += 20;
            now = t;
            detector.process(s);
            feedback.updateMotion(calibration.calibratedTiltX(), calibration.calibratedTiltY(),
                                  mouse.isScrollWheelActive() ? mouse.getAngle() : static_cast<int8_t>(0));
        }

        int sumDx() const {
            int v = 0;
            for (const auto &r: sent) v += static_cast<int8_t>(r[1]);
            return v;
        }

        int sumDy() const {
            int v = 0;
            for (const auto &r: sent) v += static_cast<int8_t>(r[2]);
            return v;
        }

        int sumWheel() const {
            int v = 0;
            for (const auto &r: sent) v += static_cast<int8_t>(r[3]);
            return v;
        }
    };

    {
        Chain c; // tilt right only
        for (int i = 0; i < 5; ++i) c.feed(+3.0f, 0, 0);
        TEST_ASSERT_GREATER_THAN_INT(0, c.sumDx());
        TEST_ASSERT_EQUAL_INT(0, c.sumDy());
        TEST_ASSERT_EQUAL_INT(0, c.sumWheel());
        TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, c.feedback.snapshot().tiltX); // ball right, same sense
    }
    {
        Chain c; // tilt left only
        for (int i = 0; i < 5; ++i) c.feed(-3.0f, 0, 0);
        TEST_ASSERT_LESS_THAN_INT(0, c.sumDx());
        TEST_ASSERT_EQUAL_INT(0, c.sumDy());
    }
    {
        Chain c; // calibratedY > 0 only
        for (int i = 0; i < 5; ++i) c.feed(0, +3.0f, 0);
        TEST_ASSERT_EQUAL_INT(0, c.sumDx());
        TEST_ASSERT_LESS_THAN_INT(0, c.sumDy()); // HID Y negated
        TEST_ASSERT_LESS_THAN_FLOAT(0.0f, c.feedback.snapshot().tiltY); // ball toward top row (LED convention, unchanged)
    }
    {
        Chain c; // calibratedY < 0 only
        for (int i = 0; i < 5; ++i) c.feed(0, -3.0f, 0);
        TEST_ASSERT_GREATER_THAN_INT(0, c.sumDy()); // HID Y negated
    }
    {
        Chain c; // +Z rotation
        for (int i = 0; i < 50; ++i) c.feed(0, 0, +30.0f);
        TEST_ASSERT_EQUAL_INT(-1, c.feedback.snapshot().scrollDirection); // confirmed chevron: DOWN
        TEST_ASSERT_LESS_THAN_INT(0, c.sumWheel()); // HID wheel: DOWN, agrees with the chevron
        TEST_ASSERT_EQUAL_INT(0, c.sumDx());
        TEST_ASSERT_EQUAL_INT(0, c.sumDy());
    }
    {
        Chain c; // -Z rotation
        for (int i = 0; i < 50; ++i) c.feed(0, 0, -30.0f);
        TEST_ASSERT_EQUAL_INT(+1, c.feedback.snapshot().scrollDirection); // confirmed chevron: UP
        TEST_ASSERT_GREATER_THAN_INT(0, c.sumWheel()); // HID wheel: UP
    }
    // Negation never overflows int8_t.
    TEST_ASSERT_EQUAL_INT(127, MouseHidReportBuilder::hidWheelFromTicks(INT8_MIN));
    TEST_ASSERT_EQUAL_INT(-127, MouseHidReportBuilder::hidWheelFromTicks(127));
    TEST_ASSERT_EQUAL_INT(1, MouseHidReportBuilder::hidWheelFromTicks(-1));
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0); // keep Unity output even if a sanitizer aborts the run
    UNITY_BEGIN();
    RUN_TEST(test_left_click_emits_down_then_up);
    RUN_TEST(test_right_click_emits_down_then_up);
    RUN_TEST(test_middle_click_emits_down_then_up);
    RUN_TEST(test_gyro_below_threshold_is_not_a_click);
    RUN_TEST(test_slow_return_keeps_button_held_for_drag);
    RUN_TEST(test_release_lockout_blocks_immediate_re_press);
    RUN_TEST(test_no_on_device_double_click);
    RUN_TEST(test_movement_deltas_follow_calibrated_tilt_with_hysteresis);
    RUN_TEST(test_movement_acceleration_curve_shape);
    RUN_TEST(test_movement_curve_preserves_sign_on_both_axes);
    RUN_TEST(test_movement_full_scale_matches_ball_full_scale);
    RUN_TEST(test_scroll_ticks_have_direction_and_rate);
    RUN_TEST(test_scroll_engage_disengage_has_a_real_deadzone);
    RUN_TEST(test_scroll_engage_disengage_thresholds_match_mouse_hysteresis);
    RUN_TEST(test_movement_start_stop_thresholds_match_intended_deadzone);
    RUN_TEST(test_scroll_suppressed_while_button_held_and_buttons_suppressed_while_scrolling);
    RUN_TEST(test_scroll_engages_despite_accompanying_side_tilt);
    RUN_TEST(test_active_scroll_fully_suppresses_movement);
    RUN_TEST(test_scroll_engages_with_tilt_that_moved_under_old_thresholds);
    RUN_TEST(test_drag_sequence_produces_down_moves_up);
    RUN_TEST(test_hid_report_layout);
    RUN_TEST(test_report_builder_one_report_per_event_with_button_mask);
    RUN_TEST(test_full_chain_gestures_to_hid_reports_via_fanout);
    RUN_TEST(test_hid_sign_mapping_matches_confirmed_display_sense);
    return UNITY_END();
}
