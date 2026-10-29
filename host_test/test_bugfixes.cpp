// Regression tests for the two bugs fixed in ticket #1181:
//  1. movement/scroll deltas were narrowed to int8_t BEFORE clamping -> wrap-around on hard tilts
//  2. MouseButtonStateMachine stored `const Button &` bound to a temporary -> dangling reference
#include "test_support.hpp"

#include <MouseButtonStateMachine.hpp>
#include <cmath>
#include <limits>
#include <type_traits>
#include <cstdio>
#include <unity.h>

using namespace Hilton;
using namespace TestSupport;

void setUp() {
}

void tearDown() {
}

// ---------------- Bug 1: clamp before narrowing ----------------

static void test_saturate_in_range_values_unchanged() {
    TEST_ASSERT_EQUAL_INT(0, Mouse::saturateToReport(0.0f));
    TEST_ASSERT_EQUAL_INT(13, Mouse::saturateToReport(13.9f)); // truncates toward zero like the old cast
    TEST_ASSERT_EQUAL_INT(-13, Mouse::saturateToReport(-13.9f));
    TEST_ASSERT_EQUAL_INT(126, Mouse::saturateToReport(126.99f));
    TEST_ASSERT_EQUAL_INT(127, Mouse::saturateToReport(127.0f));
    TEST_ASSERT_EQUAL_INT(-127, Mouse::saturateToReport(-127.0f));
}

static void test_saturate_past_overflow_point_clamps_instead_of_wrapping() {
    // What the old code effectively did on the device: 200 -> -56 (wrap). Demonstrated with a
    // well-defined int intermediate so this test itself has no UB.
    TEST_ASSERT_EQUAL_INT(-56, static_cast<int8_t>(static_cast<int>(200.0f)));

    TEST_ASSERT_EQUAL_INT(127, Mouse::saturateToReport(127.5f));
    TEST_ASSERT_EQUAL_INT(127, Mouse::saturateToReport(128.0f));
    TEST_ASSERT_EQUAL_INT(127, Mouse::saturateToReport(200.0f));
    TEST_ASSERT_EQUAL_INT(127, Mouse::saturateToReport(1.0e9f));
    TEST_ASSERT_EQUAL_INT(127, Mouse::saturateToReport(std::numeric_limits<float>::infinity()));
    // Negative side clamps to -127 (not -128: the HID logical minimum is -127).
    TEST_ASSERT_EQUAL_INT(-127, Mouse::saturateToReport(-128.0f));
    TEST_ASSERT_EQUAL_INT(-127, Mouse::saturateToReport(-300.0f));
    TEST_ASSERT_EQUAL_INT(-127, Mouse::saturateToReport(-std::numeric_limits<float>::infinity()));
    TEST_ASSERT_EQUAL_INT(0, Mouse::saturateToReport(std::numeric_limits<float>::quiet_NaN()));
}

static void test_hard_tilt_moves_cursor_at_full_speed_in_the_same_direction() {
    // Sweep accel X from 0 up to the sensor's +/-2G full scale (19.6 m/s^2), crossing the old
    // overflow point (127 / (127/9.8) = 9.8 m/s^2). dx must be monotonic non-decreasing and
    // never change sign, then saturate at +127.
    for (const float sign: {1.0f, -1.0f}) {
        Rig rig;
        rig.step(); // seed calibration at level
        int previous = 0;
        for (float a = 0.0f; a <= 19.6f; a += 0.2f) {
            rig.ax = sign * a;
            rig.sink.clear();
            rig.step();
            int dx = 0;
            for (const auto &e: rig.sink.events) {
                if (e.type == RecordedEvent::Type::Move) dx = e.dx;
            }
            if (dx != 0) {
                TEST_ASSERT_TRUE_MESSAGE(sign > 0 ? dx > 0 : dx < 0, "cursor direction flipped (wrap-around)");
                TEST_ASSERT_TRUE_MESSAGE(std::abs(dx) >= std::abs(previous), "dx not monotonic with tilt");
                previous = dx;
            }
        }
        TEST_ASSERT_EQUAL_INT(sign > 0 ? 127 : -127, previous);
    }
}

static void test_hard_yaw_scrolls_in_the_same_direction() {
    // 90 degrees of yaw * 127/60 = 190 report units -> used to wrap negative. Now saturates.
    Rig rig;
    rig.step();
    rig.yaw = 90.0f;
    rig.run(1000);
    TEST_ASSERT_EQUAL_INT(127, rig.mouse.getAngle());
    TEST_ASSERT_GREATER_THAN_INT(0, rig.sink.scrollTotal());

    Rig neg;
    neg.step();
    neg.yaw = -90.0f;
    neg.run(1000);
    TEST_ASSERT_EQUAL_INT(-127, neg.mouse.getAngle());
    TEST_ASSERT_LESS_THAN_INT(0, neg.sink.scrollTotal());
}

// ---------------- Bug 2: dangling Button reference ----------------

// A state machine holding a reference member cannot be copy-assigned; one holding the value can.
static_assert(std::is_copy_assignable_v<MouseButtonStateMachine>,
              "MouseButtonStateMachine must store Button by value (a reference member deletes copy-assignment)");

static MouseButtonStateMachine makeFromTemporary(const MouseButtonStateMachine::Button b) {
    // `b` (and the temporary the caller passed) die when this returns; with the old
    // `const Button &button` member the returned object referenced dead stack memory.
    return MouseButtonStateMachine(b);
}

static void clobberStack() {
    volatile int junk[256];
    for (int i = 0; i < 256; ++i) junk[i] = 0x5A5A5A5A ^ i;
    (void) junk[0];
}

static void test_button_survives_its_constructor_argument() {
    MouseButtonStateMachine left = makeFromTemporary(MouseButtonStateMachine::Button::Left);
    MouseButtonStateMachine right = makeFromTemporary(MouseButtonStateMachine::Button::Right);
    MouseButtonStateMachine middle{MouseButtonStateMachine::Button::Middle}; // temporary enum prvalue
    clobberStack();
    // Under ASan the old code fails right here (stack-use-after-return / -scope).
    TEST_ASSERT_EQUAL_STRING("Left", left.getButtonName().c_str());
    TEST_ASSERT_EQUAL_STRING("Right", right.getButtonName().c_str());
    TEST_ASSERT_EQUAL_STRING("Middle", middle.getButtonName().c_str());
    TEST_ASSERT_TRUE(left.getButton() == MouseButtonStateMachine::Button::Left);
}

static void test_mouse_buttons_keep_their_identity() {
    // Mouse's constructor passes temporaries -- exactly the original bug's trigger.
    Mouse mouse;
    clobberStack();
    TEST_ASSERT_EQUAL_STRING("Left", mouse.getLeftButton().getButtonName().c_str());
    TEST_ASSERT_EQUAL_STRING("Right", mouse.getRightButton().getButtonName().c_str());
    TEST_ASSERT_EQUAL_STRING("Middle", mouse.getMiddleButton().getButtonName().c_str());

    // Copies (now possible) keep their identity too.
    MouseButtonStateMachine copy = mouse.getRightButton();
    copy = mouse.getMiddleButton();
    TEST_ASSERT_EQUAL_STRING("Middle", copy.getButtonName().c_str());
}

static void test_state_machine_transitions() {
    MouseButtonStateMachine b{MouseButtonStateMachine::Button::Left};
    TEST_ASSERT_TRUE(b.getState() == MouseButtonStateMachine::State::Idle);
    b.handleEvent(MouseButtonStateMachine::Event::MouseUp); // ignored while idle
    TEST_ASSERT_TRUE(b.getState() == MouseButtonStateMachine::State::Idle);
    b.handleEvent(MouseButtonStateMachine::Event::MouseDown);
    TEST_ASSERT_TRUE(b.getState() == MouseButtonStateMachine::State::Pressed);
    b.handleEvent(MouseButtonStateMachine::Event::MouseDown); // ignored while pressed
    TEST_ASSERT_TRUE(b.getState() == MouseButtonStateMachine::State::Pressed);
    b.handleEvent(MouseButtonStateMachine::Event::MouseUp);
    TEST_ASSERT_TRUE(b.getState() == MouseButtonStateMachine::State::Idle);
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0); // keep Unity output even if a sanitizer aborts the run
    UNITY_BEGIN();
    RUN_TEST(test_saturate_in_range_values_unchanged);
    RUN_TEST(test_saturate_past_overflow_point_clamps_instead_of_wrapping);
    RUN_TEST(test_hard_tilt_moves_cursor_at_full_speed_in_the_same_direction);
    RUN_TEST(test_hard_yaw_scrolls_in_the_same_direction);
    RUN_TEST(test_button_survives_its_constructor_argument);
    RUN_TEST(test_mouse_buttons_keep_their_identity);
    RUN_TEST(test_state_machine_transitions);
    return UNITY_END();
}
