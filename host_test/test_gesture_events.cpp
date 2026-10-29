// The gesture -> event-sink seam: simulated IMU input in, discrete events out, via a fake
// sink (no hardware, no BLE). Also covers HID report encoding and the fan-out.
#include "test_support.hpp"

#include <MouseHidReport.hpp>
#include <cstdio>
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
    rig.run(500);
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
    rig.run(500);
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
    rig.run(500);
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
    rig.run(500);
    rig.spike(-99.0f, 0.0f);
    rig.spike(0.0f, -100.0f); // threshold is strictly greater than 100
    rig.spike(0.0f, +150.0f); // pitch BACK never presses Middle
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(rig.sink.events.size()));
}

static void test_slow_return_keeps_button_held_for_drag() {
    Rig rig;
    rig.run(500);
    rig.spike(-150.0f, 0.0f);
    rig.gx = 40.0f; // slow roll back: below threshold, must NOT release
    rig.run(2000);
    TEST_ASSERT_EQUAL_INT(1, rig.sink.count(Type::Down, Button::Left));
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Up));
    TEST_ASSERT_TRUE(rig.mouse.isButtonPressed(Button::Left));
}

static void test_release_lockout_blocks_immediate_re_press() {
    Rig rig;
    rig.run(500);
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
    rig.run(500);
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
    rig.run(500);

    rig.ax = 0.9f; // 11.7 units: below start (13)
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Move));

    // The sub-threshold lean above was idle, so the tilt baseline crept toward 0.9 a little
    // (that is the slow calibration doing its job); deltas are relative to that baseline,
    // which is frozen from here on because the board is moving.
    const float baseX = rig.calibration.tiltBaselineX();
    TEST_ASSERT_TRUE(baseX > 0.0f && baseX < 0.1f);
    rig.ax = 3.0f; // ~38 units
    rig.ay = -1.5f; // -19.4 units
    rig.sink.clear();
    rig.run(200); // 10 samples
    TEST_ASSERT_EQUAL_INT(10, rig.sink.count(Type::Move));
    const int expectedDx = static_cast<int>((3.0f - baseX) * GestureDetector::GRAVITY_SCALE);
    TEST_ASSERT_INT_WITHIN(1, 37, expectedDx);
    for (const auto &e: rig.sink.events) {
        TEST_ASSERT_EQUAL_INT(expectedDx, e.dx);
        TEST_ASSERT_EQUAL_INT(-19, e.dy);
    }
    TEST_ASSERT_EQUAL_FLOAT(baseX, rig.calibration.tiltBaselineX());

    rig.ax = 0.6f; // ~7 units: between stop (5) and start (13) -> keeps moving
    rig.ay = 0.0f;
    rig.sink.clear();
    rig.run(100);
    TEST_ASSERT_EQUAL_INT(5, rig.sink.count(Type::Move));
    TEST_ASSERT_EQUAL_INT(static_cast<int>((0.6f - baseX) * GestureDetector::GRAVITY_SCALE), rig.sink.events.back().dx);

    rig.ax = 0.2f; // 2.6 units: below stop
    rig.sink.clear();
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Move));
    TEST_ASSERT_FALSE(rig.mouse.isMoving());
}

static void test_scroll_ticks_have_direction_and_rate() {
    Rig rig;
    rig.run(500);
    rig.yaw = 30.0f; // 63 units -> 18.9 ticks/s
    rig.run(1000);
    TEST_ASSERT_INT_WITHIN(1, 19, rig.sink.scrollTotal());
    for (const auto &e: rig.sink.events) TEST_ASSERT_TRUE(e.type == Type::Scroll && e.ticks > 0);

    rig.yaw = 0.0f; // back to centre: stops
    rig.run(200);
    rig.sink.clear();
    rig.run(500);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.scrollTotal());

    rig.yaw = -30.0f;
    rig.run(1000);
    TEST_ASSERT_INT_WITHIN(1, -19, rig.sink.scrollTotal());

    // Below the start threshold (16 units ~ 7.6 deg): no scrolling at all.
    Rig quiet;
    quiet.run(500);
    quiet.yaw = 7.0f;
    quiet.run(2000);
    TEST_ASSERT_EQUAL_INT(0, quiet.sink.count(Type::Scroll));
}

static void test_scroll_suppressed_while_button_held_and_buttons_suppressed_while_scrolling() {
    Rig rig;
    rig.run(500);
    rig.spike(-150.0f, 0.0f); // left down
    rig.yaw = 30.0f;
    rig.run(1000);
    TEST_ASSERT_EQUAL_INT(0, rig.sink.count(Type::Scroll));

    Rig rig2;
    rig2.run(500);
    rig2.yaw = 30.0f;
    rig2.run(200);
    rig2.spike(-150.0f, 0.0f); // ignored: scroll has priority
    rig2.run(200);
    TEST_ASSERT_EQUAL_INT(0, rig2.sink.count(Type::Down));
    TEST_ASSERT_GREATER_THAN_INT(0, rig2.sink.count(Type::Scroll));
}

static void test_drag_sequence_produces_down_moves_up() {
    Rig rig;
    rig.run(500);
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
    TEST_ASSERT_EQUAL_INT(-2, static_cast<int8_t>(sent[3][3]));
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
    for (int i = 0; i < 10; ++i) feed(0, 0, 0);
    feed(-150, 0, 0); // left down
    for (int i = 0; i < 5; ++i) feed(0, 0, 0);
    feed(+150, 0, 0); // left up
    for (int i = 0; i < 20; ++i) feed(0, 0, 0);
    for (int i = 0; i < 5; ++i) feed(0, 3.0f, 0); // move right
    for (int i = 0; i < 20; ++i) feed(0, 0, 0);
    for (int i = 0; i < 50; ++i) feed(0, 0, 30.0f); // scroll up

    TEST_ASSERT_EQUAL_HEX8(0x01, sent.at(0)[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, sent.at(1)[0]);
    TEST_ASSERT_EQUAL_INT(38, static_cast<int8_t>(sent.at(2)[1]));
    int wheel = 0;
    for (const auto &r: sent) wheel += static_cast<int8_t>(r[3]);
    TEST_ASSERT_INT_WITHIN(1, 19, wheel);
    // Every event reached both sinks.
    TEST_ASSERT_EQUAL_INT(1, led.count(Type::Down));
    TEST_ASSERT_EQUAL_INT(1, led.count(Type::Up));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(sent.size()), static_cast<int>(led.events.size()));
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
    RUN_TEST(test_scroll_ticks_have_direction_and_rate);
    RUN_TEST(test_scroll_suppressed_while_button_held_and_buttons_suppressed_while_scrolling);
    RUN_TEST(test_drag_sequence_produces_down_moves_up);
    RUN_TEST(test_hid_report_layout);
    RUN_TEST(test_report_builder_one_report_per_event_with_button_mask);
    RUN_TEST(test_full_chain_gestures_to_hid_reports_via_fanout);
    return UNITY_END();
}
