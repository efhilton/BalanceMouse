// LED feedback: the PURE renderer (state + time -> 64 pixels), the matrix mapping, the
// FeedbackState tracker, and -- most importantly -- the intensity ceiling everywhere
// (Lights::MAX_INTENSITY_PERCENT; lowered 50% -> 25% -> 12% in #1181 after two real-hardware heat findings).
#include <FeedbackRenderer.hpp>
#include <FeedbackState.hpp>
#include <LightPixel.hpp>
#include <MatrixLayout.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <unity.h>
#include <vector>

using namespace Hilton;
using Mode = FeedbackRenderer::Mode;
using R = FeedbackRenderer;

void setUp() {
}

void tearDown() {
}

// ---------------- helpers ----------------

static FeedbackSnapshot connected() {
    FeedbackSnapshot s{};
    s.phase = ConnectionPhase::Connected;
    return s;
}

static int litCount(const Frame &f) {
    int n = 0;
    for (const auto &p: f.raw()) n += p.isLit() ? 1 : 0;
    return n;
}

static bool lit(const Frame &f, const int x, const int y) { return f.at(x, y).isLit(); }

static bool isColor(const RgbPixel &p, const RgbPixel &c) {
    return p.r == c.r && p.g == c.g && p.b == c.b;
}

static bool columnIs(const Frame &f, const int x, const RgbPixel &c) {
    for (int y = 0; y < 8; ++y) {
        if (!isColor(f.at(x, y), c) || !f.at(x, y).isLit()) return false;
    }
    return true;
}

static bool rowIs(const Frame &f, const int y, const RgbPixel &c) {
    for (int x = 0; x < 8; ++x) {
        if (!isColor(f.at(x, y), c) || !f.at(x, y).isLit()) return false;
    }
    return true;
}

static bool allAre(const Frame &f, const RgbPixel &c) {
    for (const auto &p: f.raw()) {
        if (!p.isLit() || !isColor(p, c)) return false;
    }
    return true;
}

static int maxIntensity(const Frame &f) {
    int m = 0;
    for (const auto &p: f.raw()) m = std::max(m, static_cast<int>(p.intensity));
    return m;
}

/** Assert the ceiling for one frame (used by every test that renders). */
static Frame renderChecked(const FeedbackSnapshot &s, const uint32_t t) {
    Frame f = R::render(s, t);
    TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE(Lights::MAX_INTENSITY_PERCENT, maxIntensity(f),
                                          "pixel intensity above the ceiling");
    return f;
}

// ---------------- matrix mapping ----------------

static void test_matrix_mapping_is_progressive_row_major() {
    TEST_ASSERT_EQUAL_INT(0, Matrix::chainIndex(0, 0));
    TEST_ASSERT_EQUAL_INT(7, Matrix::chainIndex(7, 0));
    TEST_ASSERT_EQUAL_INT(8, Matrix::chainIndex(0, 1)); // row 1 starts on the LEFT again (not serpentine)
    TEST_ASSERT_EQUAL_INT(15, Matrix::chainIndex(7, 1));
    TEST_ASSERT_EQUAL_INT(63, Matrix::chainIndex(7, 7));
    TEST_ASSERT_EQUAL_INT(-1, Matrix::chainIndex(8, 0));
    TEST_ASSERT_EQUAL_INT(-1, Matrix::chainIndex(0, -1));
    std::set<int> seen;
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x) seen.insert(Matrix::chainIndex(x, y));
    TEST_ASSERT_EQUAL_INT(64, static_cast<int>(seen.size()));
    TEST_ASSERT_EQUAL_INT(0, *seen.begin());
    TEST_ASSERT_EQUAL_INT(63, *seen.rbegin());
}

// ---------------- ball ----------------

static void test_ball_is_centred_1x1_at_rest() {
    // 1x1 ball (#1181, Edgar). 8-wide grid has no exact centre pixel; std::lround's
    // round-half-away-from-zero puts the centred ball at (4, 4), not (3, 3).
    const Frame f = renderChecked(connected(), 5000);
    TEST_ASSERT_EQUAL_INT(1, litCount(f));
    TEST_ASSERT_TRUE(lit(f, 4, 4));
    TEST_ASSERT_TRUE(isColor(f.at(4, 4), R::BALL_WHITE));
}

static void test_ball_follows_calibrated_tilt() {
    // 1x1 ball, full travel 0..7 (#1181, Edgar). Rest/NaN land on (4, 4), not (3, 3) -- see
    // test_ball_is_centred_1x1_at_rest for why.
    constexpr float FULL = R::BALL_FULL_SCALE_TILT;
    int x = 0, y = 0;
    R::ballPosition(0.0f, 0.0f, x, y);
    TEST_ASSERT_EQUAL_INT(4, x);
    TEST_ASSERT_EQUAL_INT(4, y);
    R::ballPosition(+FULL, 0.0f, x, y); // lean right (X channel), full scale -> right edge
    TEST_ASSERT_EQUAL_INT(7, x);
    R::ballPosition(-FULL, 0.0f, x, y);
    TEST_ASSERT_EQUAL_INT(0, x);
    R::ballPosition(0.0f, +FULL, x, y); // +y is "down", same as the cursor
    TEST_ASSERT_EQUAL_INT(7, y);
    R::ballPosition(0.0f, -FULL, x, y);
    TEST_ASSERT_EQUAL_INT(0, y);
    R::ballPosition(+FULL / 2.0f, -FULL / 2.0f, x, y); // partial: half scale, 3.5 +/- 1.75
    TEST_ASSERT_EQUAL_INT(5, x);
    TEST_ASSERT_EQUAL_INT(2, y);
    R::ballPosition(+100.0f, -100.0f, x, y); // way past full scale: clamps, never leaves the grid
    TEST_ASSERT_EQUAL_INT(7, x);
    TEST_ASSERT_EQUAL_INT(0, y);
    R::ballPosition(NAN, NAN, x, y);
    TEST_ASSERT_EQUAL_INT(4, x);
    TEST_ASSERT_EQUAL_INT(4, y);

    // Monotonic: more lean right (X) never moves the ball left.
    int prev = -1;
    for (float t = -6.0f; t <= 6.0f; t += 0.1f) {
        R::ballPosition(t, 0.0f, x, y);
        TEST_ASSERT_TRUE(x >= prev);
        prev = x;
    }

    FeedbackSnapshot s = connected();
    s.tiltX = FULL;
    const Frame f = renderChecked(s, 0);
    TEST_ASSERT_TRUE(lit(f, 7, 4));
    TEST_ASSERT_TRUE(isColor(f.at(7, 4), R::BALL_WHITE));
    TEST_ASSERT_EQUAL_INT(1, litCount(f));
}

/**
 * End-to-end check of Edgar's precise ball-direction spec (#1181, right-hand rule, confirmed
 * bars at +Y=top/-X=left/+X=right): tilt -X -> ball +Y, tilt +X -> ball -Y, tilt +Y -> ball +X,
 * tilt -Y -> ball -X. Drives FeedbackState::updateMotion() with calibratedTiltX/Y values in
 * their REAL physical sign (as established on real hardware), not display-space values, so this
 * pins the whole updateMotion() + ballPosition() display path rather than just the geometry.
 * The physical signs are anchored empirically on Edgar's real-hardware tests (2026-11-02):
 *   - X (round 2, both channels negated -> +Y rotation sent the ball LEFT; round 3, no
 *     correction -> "+/- X direction is correct"): calibratedTiltX is POSITIVE under +Y
 *     rotation, NEGATIVE under -Y rotation.
 *   - Y (round 3, no correction -> forward (-X) tilt sent the ball BACKWARDS/down):
 *     calibratedTiltY is POSITIVE under -X (forward) rotation, NEGATIVE under +X rotation.
 * Hence updateMotion() passes X through and negates Y. Any sign flip on either channel in
 * updateMotion() fails the matching cases below.
 */
static void test_ball_moves_per_edgars_right_hand_rule_spec() {
    uint32_t now = 0;
    auto ballFor = [&](const float calibratedTiltX, const float calibratedTiltY) {
        FeedbackState st([&now] { return now; });
        st.updateMotion(calibratedTiltX, calibratedTiltY, 0);
        int x = 0, y = 0;
        R::ballPosition(st.snapshot().tiltX, st.snapshot().tiltY, x, y);
        return std::pair{x, y};
    };
    constexpr int CENTER{4}; // 1x1 ball rest position (#1181) -- see test_ball_is_centred_1x1_at_rest.

    // Tilt +Y: calibratedTiltX is POSITIVE -> ball must move +X (right, x > centre).
    auto [xPlusY, yPlusY] = ballFor(+3.0f, 0.0f);
    TEST_ASSERT_TRUE(xPlusY > CENTER);
    TEST_ASSERT_EQUAL_INT(CENTER, yPlusY);

    // Tilt -Y: calibratedTiltX is NEGATIVE -> ball must move -X (left, x < centre).
    auto [xMinusY, yMinusY] = ballFor(-3.0f, 0.0f);
    TEST_ASSERT_TRUE(xMinusY < CENTER);
    TEST_ASSERT_EQUAL_INT(CENTER, yMinusY);

    // Tilt -X (Edgar's "forward tilt"): calibratedTiltY is POSITIVE -> ball must move +Y
    // (forward/up, toward row 0, y < centre).
    auto [xMinusX, yMinusX] = ballFor(0.0f, +3.0f);
    TEST_ASSERT_EQUAL_INT(CENTER, xMinusX);
    TEST_ASSERT_TRUE(yMinusX < CENTER);

    // Tilt +X: calibratedTiltY is NEGATIVE -> ball must move -Y (down, toward row 7, y > centre).
    auto [xPlusX, yPlusX] = ballFor(0.0f, -3.0f);
    TEST_ASSERT_EQUAL_INT(CENTER, xPlusX);
    TEST_ASSERT_TRUE(yPlusX > CENTER);

    // Full scale (20 deg -> edge, #1181): -X tilt reaches the TOP row exactly, +X the bottom row;
    // +Y tilt reaches the RIGHT column exactly, -Y the left column.
    auto [xTop, yTop] = ballFor(0.0f, +R::BALL_FULL_SCALE_TILT);
    TEST_ASSERT_EQUAL_INT(0, yTop);
    auto [xBottom, yBottom] = ballFor(0.0f, -R::BALL_FULL_SCALE_TILT);
    TEST_ASSERT_EQUAL_INT(Hilton::Matrix::HEIGHT - 1, yBottom);
    auto [xRight, yRight] = ballFor(+R::BALL_FULL_SCALE_TILT, 0.0f);
    TEST_ASSERT_EQUAL_INT(Hilton::Matrix::WIDTH - 1, xRight);
    auto [xLeft, yLeft] = ballFor(-R::BALL_FULL_SCALE_TILT, 0.0f);
    TEST_ASSERT_EQUAL_INT(0, xLeft);
    (void) xTop;
    (void) xBottom;
    (void) yRight;
    (void) yLeft;

    // Corner case: centred at rest (zero tilt on both channels) stays at centre.
    auto [xRest, yRest] = ballFor(0.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT(CENTER, xRest);
    TEST_ASSERT_EQUAL_INT(CENTER, yRest);

    // Corner case: a simultaneous +Y and +X tilt (diagonal; calibratedTiltX > 0, calibratedTiltY
    // < 0) moves +X and -Y together, not just whichever channel happens to be checked first.
    auto [xDiag, yDiag] = ballFor(+3.0f, -3.0f);
    TEST_ASSERT_TRUE(xDiag > CENTER);
    TEST_ASSERT_TRUE(yDiag > CENTER);
}

/**
 * Cross-checks FeedbackRenderer::BALL_FULL_SCALE_TILT against its own documented derivation
 * from BALL_FULL_SCALE_TILT_DEG (#1181, Edgar: 20 degrees should reach the matrix edge), the
 * same consistency-guard pattern as the scroll engage/disengage thresholds -- so a future "just
 * change the degree constant" edit can't silently leave the m/s^2 value stale.
 */
static void test_ball_full_scale_tilt_matches_its_degree_spec() {
    constexpr float PI{3.14159265358979f};
    constexpr float STANDARD_GRAVITY_MS2{9.81f};
    const float expected = STANDARD_GRAVITY_MS2 * std::sin(R::BALL_FULL_SCALE_TILT_DEG * PI / 180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, R::BALL_FULL_SCALE_TILT);
}

// ---------------- buttons: click (solid) vs drag (blinking) ----------------

static void test_click_indicators_are_solid() {
    for (int b = 0; b < 3; ++b) {
        FeedbackSnapshot s = connected();
        s.buttonPressed[b] = true;
        s.buttonPressedAtMs[b] = 10000;
        for (uint32_t t = 10000; t < 10000 + R::DRAG_DWELL_MS; t += 10) {
            const Frame f = renderChecked(s, t);
            if (b == 0) {
                TEST_ASSERT_TRUE(columnIs(f, 0, R::LEFT_GREEN));
                TEST_ASSERT_TRUE(isColor(f.at(0, 0), RgbPixel{0, 255, 0, 0})); // left = green (ticket)
            }
            if (b == 1) TEST_ASSERT_TRUE(columnIs(f, 7, R::RIGHT_AMBER));
            if (b == 2) TEST_ASSERT_TRUE(rowIs(f, 0, R::MIDDLE_CYAN));
            TEST_ASSERT_TRUE(lit(f, 4, 4)); // ball still visible underneath (1x1 rest position, #1181)
        }
    }
    // Nothing pressed -> no indicator.
    const Frame idle = renderChecked(connected(), 10000);
    TEST_ASSERT_FALSE(lit(idle, 0, 0));
    TEST_ASSERT_FALSE(lit(idle, 7, 7));
}

static void test_drag_indicators_blink() {
    for (int b = 0; b < 3; ++b) {
        FeedbackSnapshot s = connected();
        s.buttonPressed[b] = true;
        s.buttonPressedAtMs[b] = 20000;
        auto indicatorOn = [&](const uint32_t t) {
            const Frame f = renderChecked(s, t);
            return b == 0 ? columnIs(f, 0, R::LEFT_GREEN) : b == 1 ? columnIs(f, 7, R::RIGHT_AMBER) : rowIs(f, 0, R::MIDDLE_CYAN);
        };
        const uint32_t d = 20000 + R::DRAG_DWELL_MS;
        TEST_ASSERT_TRUE(indicatorOn(d + 50)); // first blink phase is on
        TEST_ASSERT_FALSE(indicatorOn(d + R::DRAG_BLINK_HALF_PERIOD_MS + 50));
        TEST_ASSERT_TRUE(indicatorOn(d + 2 * R::DRAG_BLINK_HALF_PERIOD_MS + 50));
        // Over 4 s of drag it toggles regularly: count on->off transitions.
        int toggles = 0;
        bool prev = indicatorOn(d);
        for (uint32_t t = d; t < d + 4000; t += 10) {
            const bool now = indicatorOn(t);
            toggles += (prev && !now) ? 1 : 0;
            prev = now;
        }
        TEST_ASSERT_INT_WITHIN(1, 4000 / (2 * R::DRAG_BLINK_HALF_PERIOD_MS), toggles);
    }
    TEST_ASSERT_FALSE(R::isDrag(1000, 1000 + R::DRAG_DWELL_MS - 1));
    TEST_ASSERT_TRUE(R::isDrag(1000, 1000 + R::DRAG_DWELL_MS));
}

// ---------------- scroll chevrons ----------------

static int apexRow(const Frame &f) {
    // Apex cells (3,y) and (4,y) lit while (2,y) is not; return the smallest such y.
    for (int y = 0; y < 8; ++y) {
        if (lit(f, 3, y) && lit(f, 4, y) && !lit(f, 2, y)) return y;
    }
    return -1;
}

static void test_scroll_chevrons_animate_in_scroll_direction() {
    for (const int dir: {+1, -1}) {
        FeedbackSnapshot s = connected();
        s.scrollDirection = static_cast<int8_t>(dir);
        const Frame f0 = renderChecked(s, 0);
        // Chevrons replace the ball and use their own colour.
        TEST_ASSERT_TRUE(isColor(f0.at(3, apexRow(f0)), R::CHEVRON_PURPLE));
        TEST_ASSERT_EQUAL_INT(8, litCount(f0)); // two chevrons (apex row + arm row, 2 px each) in 8 rows

        // Genuinely animated: consecutive steps differ, the cycle repeats after PERIOD steps.
        std::vector<Frame> frames;
        for (int k = 0; k < R::CHEVRON_PERIOD_ROWS; ++k) frames.push_back(renderChecked(s, k * R::CHEVRON_STEP_MS));
        for (int k = 1; k < R::CHEVRON_PERIOD_ROWS; ++k) TEST_ASSERT_TRUE(frames[k] != frames[k - 1]);
        TEST_ASSERT_TRUE(renderChecked(s, R::CHEVRON_PERIOD_ROWS * R::CHEVRON_STEP_MS) == frames[0]);
        // Same frame within one step.
        TEST_ASSERT_TRUE(renderChecked(s, R::CHEVRON_STEP_MS - 1) == frames[0]);

        // Direction: the apex row moves up (y decreases, mod 4) when scrolling up, down otherwise.
        const int y0 = apexRow(frames[0]) % 4;
        const int y1 = apexRow(frames[1]) % 4;
        TEST_ASSERT_EQUAL_INT(((y0 - dir) % 4 + 4) % 4, y1);

        // Shape: pointing up = arms BELOW the apex; pointing down = arms ABOVE the apex.
        const int a = apexRow(frames[0]);
        const int armY = a + (dir > 0 ? 1 : -1);
        if (armY >= 0 && armY < 8) {
            TEST_ASSERT_TRUE(lit(frames[0], 2, armY) && lit(frames[0], 5, armY));
        }
    }
}

/**
 * Chevron animation speed scales with scroll deflection magnitude (#1181, Edgar: "start
 * animating slowly, speeding up as the angle increases, up to a max angle"). chevronStepMs is
 * the row-advance period, so SMALLER means FASTER.
 */
static void test_chevron_speed_scales_with_scroll_magnitude() {
    TEST_ASSERT_EQUAL_UINT32(R::CHEVRON_STEP_MS, R::chevronStepMs(0)); // slowest at/near engage
    TEST_ASSERT_EQUAL_UINT32(R::CHEVRON_STEP_MS_FAST, R::chevronStepMs(127)); // fastest at max

    // Monotonically faster (non-increasing period) as magnitude rises across the whole range.
    uint32_t prev = R::chevronStepMs(0);
    for (int m = 1; m <= 127; ++m) {
        const uint32_t step = R::chevronStepMs(static_cast<uint8_t>(m));
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(prev, step);
        prev = step;
    }

    // Visible end to end through render(): a near-max-deflection scroll animates measurably
    // faster (more distinct frames over the same wall-clock window) than a near-engagement one.
    FeedbackSnapshot slow = connected();
    slow.scrollDirection = 1;
    slow.scrollMagnitude = 16; // just past engagement
    FeedbackSnapshot fast = connected();
    fast.scrollDirection = 1;
    fast.scrollMagnitude = 127; // full deflection

    auto countTransitions = [](const FeedbackSnapshot &s, const uint32_t windowMs) {
        int transitions = 0;
        Frame prevFrame = renderChecked(s, 0);
        for (uint32_t t = 1; t < windowMs; t += 1) {
            const Frame f = renderChecked(s, t);
            if (f != prevFrame) ++transitions;
            prevFrame = f;
        }
        return transitions;
    };
    constexpr uint32_t WINDOW_MS{2000};
    const int slowTransitions = countTransitions(slow, WINDOW_MS);
    const int fastTransitions = countTransitions(fast, WINDOW_MS);
    TEST_ASSERT_GREATER_THAN_INT(slowTransitions, fastTransitions);
}

// ---------------- pairing pulse ----------------

static void test_pairing_pulse_breathes_blue_over_whole_matrix() {
    for (const auto phase: {ConnectionPhase::Idle, ConnectionPhase::Advertising, ConnectionPhase::Connecting}) {
        FeedbackSnapshot s{};
        s.phase = phase;
        s.buttonPressed[0] = true; // mouse state is ignored while not connected
        TEST_ASSERT_TRUE(R::modeFor(s, 0) == Mode::PairingPulse);

        int minI = 255, maxI = 0;
        int prev = -1;
        for (uint32_t t = 0; t < 2 * R::PULSE_PERIOD_MS; t += 40) { // 25 Hz, like the LED task
            const Frame f = renderChecked(s, t);
            // PULSE_MIN_INTENSITY is 0 since the second heat halving (2026-11-02, #1181), so the
            // breath legitimately bottoms out dark: every pixel must still carry the pulse colour,
            // and be lit whenever the pulse intensity is above zero.
            for (const auto &p: f.raw()) TEST_ASSERT_TRUE(isColor(p, R::PULSE_BLUE));
            const int i = f.at(0, 0).intensity;
            if (i > 0) TEST_ASSERT_TRUE(allAre(f, R::PULSE_BLUE));
            for (const auto &p: f.raw()) TEST_ASSERT_EQUAL_INT(i, p.intensity); // uniform
            if (prev >= 0) TEST_ASSERT_TRUE(std::abs(i - prev) <= 2); // smooth, no jumps
            prev = i;
            minI = std::min(minI, i);
            maxI = std::max(maxI, i);
        }
        TEST_ASSERT_EQUAL_INT(R::PULSE_MIN_INTENSITY, minI);
        TEST_ASSERT_EQUAL_INT(R::PULSE_MAX_INTENSITY, maxI);
    }
    TEST_ASSERT_EQUAL_INT(R::PULSE_MIN_INTENSITY, R::pulseIntensity(0));
    TEST_ASSERT_EQUAL_INT(R::PULSE_MAX_INTENSITY, R::pulseIntensity(R::PULSE_PERIOD_MS / 2));
    TEST_ASSERT_EQUAL_INT(R::pulseIntensity(700), R::pulseIntensity(700 + 5 * R::PULSE_PERIOD_MS)); // repeats
}

// ---------------- pairing flashes ----------------

/** Sample a flash sequence at 5 ms resolution; return how many distinct "on" bursts of `color` occur. */
static int countFlashes(const FeedbackSnapshot &s, const uint32_t from, const uint32_t to, const RgbPixel &color,
                        bool &allOffBetween) {
    int flashes = 0;
    bool prevOn = false;
    allOffBetween = true;
    for (uint32_t t = from; t < to; t += 5) {
        const Frame f = renderChecked(s, t);
        const bool on = allAre(f, color);
        if (!on && litCount(f) != 0) allOffBetween = false;
        if (on && !prevOn) ++flashes;
        prevOn = on;
    }
    return flashes;
}

static void test_pairing_success_two_green_flashes_then_mouse_display() {
    FeedbackSnapshot s = connected();
    s.pairingOutcome = PairingOutcome::Success;
    s.pairingOutcomeAtMs = 50000;

    TEST_ASSERT_TRUE(allAre(renderChecked(s, 50000), R::SUCCESS_GREEN)); // starts immediately
    bool cleanGaps = false;
    TEST_ASSERT_EQUAL_INT(2, countFlashes(s, 50000, 50000 + R::SUCCESS_FLASH_TOTAL_MS, R::SUCCESS_GREEN, cleanGaps));
    TEST_ASSERT_TRUE(cleanGaps);

    // Hand-off: right after the sequence, back to the normal mouse display (centred ball).
    TEST_ASSERT_TRUE(R::modeFor(s, 50000 + R::SUCCESS_FLASH_TOTAL_MS) == Mode::Mouse);
    const Frame after = renderChecked(s, 50000 + R::SUCCESS_FLASH_TOTAL_MS);
    TEST_ASSERT_EQUAL_INT(1, litCount(after));
    TEST_ASSERT_TRUE(lit(after, 4, 4)); // 1x1 rest position, #1181
    // And it never flashes again.
    bool dummy = false;
    TEST_ASSERT_EQUAL_INT(0, countFlashes(s, 50000 + R::SUCCESS_FLASH_TOTAL_MS, 60000, R::SUCCESS_GREEN, dummy));
}

static void test_pairing_failure_three_red_flashes_then_back_to_pulse() {
    FeedbackSnapshot s{};
    s.phase = ConnectionPhase::Advertising;
    s.pairingOutcome = PairingOutcome::Failed;
    s.pairingOutcomeAtMs = 70000;

    TEST_ASSERT_TRUE(allAre(renderChecked(s, 70000), R::FAILURE_RED));
    bool cleanGaps = false;
    TEST_ASSERT_EQUAL_INT(3, countFlashes(s, 70000, 70000 + R::FAILURE_FLASH_TOTAL_MS, R::FAILURE_RED, cleanGaps));
    TEST_ASSERT_TRUE(cleanGaps);
    // No red at all outside the window, and the blue pulse is back.
    TEST_ASSERT_TRUE(R::modeFor(s, 70000 + R::FAILURE_FLASH_TOTAL_MS) == Mode::PairingPulse);
    TEST_ASSERT_TRUE(allAre(renderChecked(s, 70000 + R::FAILURE_FLASH_TOTAL_MS), R::PULSE_BLUE));
    // Even while still "Connecting" the failure flash takes priority over the pulse.
    s.phase = ConnectionPhase::Connecting;
    TEST_ASSERT_TRUE(R::modeFor(s, 70010) == Mode::FailureFlash);
}

static void test_flash_ordering_on_off_on() {
    FeedbackSnapshot s{};
    s.phase = ConnectionPhase::Advertising;
    s.pairingOutcome = PairingOutcome::Failed;
    s.pairingOutcomeAtMs = 0;
    TEST_ASSERT_TRUE(allAre(renderChecked(s, 0), R::FAILURE_RED));
    TEST_ASSERT_EQUAL_INT(0, litCount(renderChecked(s, R::FLASH_ON_MS)));
    TEST_ASSERT_TRUE(allAre(renderChecked(s, R::FLASH_ON_MS + R::FLASH_OFF_MS), R::FAILURE_RED));
    // A timestamp from the future (clock skew) must not trigger a flash.
    s.pairingOutcomeAtMs = 1000;
    TEST_ASSERT_TRUE(R::modeFor(s, 500) == Mode::PairingPulse);
}

// ---------------- FeedbackState tracker ----------------

static void test_feedback_state_tracks_events_and_new_pairing_outcomes() {
    uint32_t now = 1000;
    FeedbackState st([&now] { return now; });

    st.onButtonDown(MouseButtonStateMachine::Button::Right);
    FeedbackSnapshot s = st.snapshot();
    TEST_ASSERT_TRUE(s.buttonPressed[1]);
    TEST_ASSERT_EQUAL_UINT32(1000, s.buttonPressedAtMs[1]);
    now = 1500;
    st.onButtonUp(MouseButtonStateMachine::Button::Right);
    TEST_ASSERT_FALSE(st.snapshot().buttonPressed[1]);

    // Ball-display X passes through unchanged, Y is negated (#1181, found empirically over three
    // rounds of Edgar's real-hardware testing; see FeedbackState::updateMotion). Mixed signs so a
    // flip on either axis fails.
    st.updateMotion(1.0f, -2.0f, 0);
    TEST_ASSERT_EQUAL_FLOAT(+1.0f, st.snapshot().tiltX);
    TEST_ASSERT_EQUAL_FLOAT(+2.0f, st.snapshot().tiltY);
    TEST_ASSERT_EQUAL_INT(0, st.snapshot().scrollDirection);
    TEST_ASSERT_EQUAL_UINT8(0, st.snapshot().scrollMagnitude);
    // angle=-40 is negative calibratedYaw (-Z): spec (#1181, Edgar) is -Z -> scroll-UP (+1),
    // not the naive angle>0 -> +1 mapping -- see FeedbackState::snapshot()'s comment.
    st.updateMotion(0, 0, -40);
    TEST_ASSERT_EQUAL_INT(+1, st.snapshot().scrollDirection);
    TEST_ASSERT_EQUAL_UINT8(40, st.snapshot().scrollMagnitude);
    st.updateMotion(0, 0, 0);
    // ticks share angle's sign (GestureDetector), so a positive tick is also +Z -> scroll-DOWN
    // (-1) during the hold window, consistent with the live-angle path above.
    st.onScroll(+1);
    TEST_ASSERT_EQUAL_INT(-1, st.snapshot().scrollDirection); // tick keeps chevrons alive briefly
    TEST_ASSERT_EQUAL_UINT8(40, st.snapshot().scrollMagnitude); // holds the last live magnitude
    now += FeedbackState::SCROLL_HOLD_MS;
    TEST_ASSERT_EQUAL_INT(0, st.snapshot().scrollDirection);
    TEST_ASSERT_EQUAL_UINT8(0, st.snapshot().scrollMagnitude);

    PairingStatus p{};
    st.updateConnection(ConnectionPhase::Advertising, p);
    TEST_ASSERT_TRUE(st.snapshot().pairingOutcome == PairingOutcome::None);
    p.outcome = PairingOutcome::Failed;
    p.sequence = 1;
    now = 5000;
    st.updateConnection(ConnectionPhase::Advertising, p);
    TEST_ASSERT_TRUE(st.snapshot().pairingOutcome == PairingOutcome::Failed);
    TEST_ASSERT_EQUAL_UINT32(5000, st.snapshot().pairingOutcomeAtMs);
    now = 6000;
    st.updateConnection(ConnectionPhase::Advertising, p); // same sequence: not a new event
    TEST_ASSERT_EQUAL_UINT32(5000, st.snapshot().pairingOutcomeAtMs);
    p.sequence = 2; // same outcome value again, but a new event -> re-timestamped
    st.updateConnection(ConnectionPhase::Connecting, p);
    TEST_ASSERT_EQUAL_UINT32(6000, st.snapshot().pairingOutcomeAtMs);
    TEST_ASSERT_TRUE(st.snapshot().phase == ConnectionPhase::Connecting);
}

// ---------------- THE INTENSITY CEILING ----------------
// Lowered 50% -> 25% in #1181 after Edgar's real-hardware pass found the unit running warmer
// than it should even while already compliant with the old 50% ceiling, then 25% -> 12% on
// 2026-11-02 after a second heat complaint.

static void test_light_pixel_clamps_intensity_at_the_source() {
    constexpr uint8_t CEIL = Lights::MAX_INTENSITY_PERCENT;
    LightPixel p;
    p.setIntensity(CEIL);
    TEST_ASSERT_EQUAL_UINT8(CEIL, p.getIntensity());
    p.setIntensity(CEIL + 1);
    TEST_ASSERT_EQUAL_UINT8(CEIL, p.getIntensity());
    p.setIntensity(100);
    TEST_ASSERT_EQUAL_UINT8(CEIL, p.getIntensity());
    p.setIntensity(255);
    TEST_ASSERT_EQUAL_UINT8(CEIL, p.getIntensity());
    p.setIntensity(0);
    TEST_ASSERT_EQUAL_UINT8(0, p.getIntensity());
    // The constructors clamp too.
    const LightPixel q(255, 255, 255, 255, 100);
    TEST_ASSERT_EQUAL_UINT8(CEIL, q.getIntensity());
    const LightPixel r(Color(255, 0, 0, 0), 99);
    TEST_ASSERT_EQUAL_UINT8(CEIL, r.getIntensity());
    // What actually goes on the wire: never more than CEIL% of full drive.
    TEST_ASSERT_EQUAL_INT(static_cast<int>(255.0f * CEIL / 100.0f), q.getScaledRed());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(255.0f * CEIL / 100.0f), q.getScaledWhite());
    for (int i = 0; i <= 255; ++i) {
        LightPixel w(255, 255, 255, 255, 0);
        w.setIntensity(static_cast<uint8_t>(i));
        TEST_ASSERT_LESS_OR_EQUAL_INT(CEIL, w.getIntensity());
        TEST_ASSERT_LESS_OR_EQUAL_INT(static_cast<int>(255.0f * CEIL / 100.0f), w.getScaledGreen());
    }
}

static void test_frame_set_clamps_intensity() {
    constexpr uint8_t CEIL = Lights::MAX_INTENSITY_PERCENT;
    Frame f;
    f.set(1, 1, RgbPixel{255, 255, 255, 100});
    TEST_ASSERT_EQUAL_UINT8(CEIL, f.at(1, 1).intensity);
    f.fill(RgbPixel{1, 2, 3, 255});
    TEST_ASSERT_EQUAL_INT(CEIL, maxIntensity(f));
    f.set(-1, 0, RgbPixel{255, 0, 0, 10}); // out of bounds: ignored, no crash
    f.set(0, 8, RgbPixel{255, 0, 0, 10});
}

static void test_no_mode_ever_exceeds_the_ceiling() {
    // Exhaustive-ish sweep over every mode and every combination of inputs, ~10 s of time at
    // 7 ms steps (co-prime with every animation period), plus a seeded random fuzz.
    long frames = 0;
    int globalMax = 0;
    auto check = [&](const FeedbackSnapshot &s, const uint32_t t) {
        const Frame f = R::render(s, t);
        const int m = maxIntensity(f);
        globalMax = std::max(globalMax, m);
        ++frames;
        TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE(Lights::MAX_INTENSITY_PERCENT, m, "pixel intensity above the ceiling");
    };

    const ConnectionPhase phases[] = {ConnectionPhase::Idle, ConnectionPhase::Advertising,
                                      ConnectionPhase::Connecting, ConnectionPhase::Connected};
    const PairingOutcome outcomes[] = {PairingOutcome::None, PairingOutcome::Success, PairingOutcome::Failed};
    const float tilts[] = {-100.0f, -R::BALL_FULL_SCALE_TILT, 0.0f, 2.0f, R::BALL_FULL_SCALE_TILT, 100.0f};
    for (const auto ph: phases) {
        for (const auto oc: outcomes) {
            for (int buttons = 0; buttons < 8; ++buttons) {
                for (int dir = -1; dir <= 1; ++dir) {
                    for (const float tilt: tilts) {
                        FeedbackSnapshot s{};
                        s.phase = ph;
                        s.pairingOutcome = oc;
                        s.pairingOutcomeAtMs = 1000;
                        for (int b = 0; b < 3; ++b) {
                            s.buttonPressed[b] = (buttons >> b) & 1;
                            s.buttonPressedAtMs[b] = 900 + 100 * b;
                        }
                        s.scrollDirection = static_cast<int8_t>(dir);
                        s.tiltX = tilt;
                        s.tiltY = -tilt;
                        for (uint32_t t = 0; t < 10000; t += 97) check(s, t);
                        for (uint32_t t = 1000; t < 2000; t += 7) check(s, t); // dense over flash windows
                    }
                }
            }
        }
    }

    uint32_t seed = 0xC0FFEE;
    auto rnd = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return seed;
    };
    for (int i = 0; i < 20000; ++i) {
        FeedbackSnapshot s{};
        s.phase = phases[rnd() % 4];
        s.pairingOutcome = outcomes[rnd() % 3];
        s.pairingOutcomeAtMs = rnd() % 100000;
        for (int b = 0; b < 3; ++b) {
            s.buttonPressed[b] = rnd() & 1;
            s.buttonPressedAtMs[b] = rnd() % 100000;
        }
        s.scrollDirection = static_cast<int8_t>(static_cast<int>(rnd() % 3) - 1);
        s.tiltX = static_cast<float>(static_cast<int>(rnd() % 4000) - 2000) / 100.0f;
        s.tiltY = static_cast<float>(static_cast<int>(rnd() % 4000) - 2000) / 100.0f;
        check(s, rnd() % 200000);
    }

    TEST_ASSERT_GREATER_THAN(100000, frames);
    TEST_ASSERT_LESS_OR_EQUAL_INT(Lights::MAX_INTENSITY_PERCENT, globalMax);
    TEST_ASSERT_GREATER_THAN_INT(0, globalMax); // sanity: something was actually lit
    printf("[%d%% ceiling] %ld frames checked across all modes, max intensity seen = %d\n",
           Lights::MAX_INTENSITY_PERCENT, frames, globalMax);
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0); // keep Unity output even if a sanitizer aborts the run
    UNITY_BEGIN();
    RUN_TEST(test_matrix_mapping_is_progressive_row_major);
    RUN_TEST(test_ball_is_centred_1x1_at_rest);
    RUN_TEST(test_ball_follows_calibrated_tilt);
    RUN_TEST(test_ball_moves_per_edgars_right_hand_rule_spec);
    RUN_TEST(test_ball_full_scale_tilt_matches_its_degree_spec);
    RUN_TEST(test_click_indicators_are_solid);
    RUN_TEST(test_drag_indicators_blink);
    RUN_TEST(test_scroll_chevrons_animate_in_scroll_direction);
    RUN_TEST(test_chevron_speed_scales_with_scroll_magnitude);
    RUN_TEST(test_pairing_pulse_breathes_blue_over_whole_matrix);
    RUN_TEST(test_pairing_success_two_green_flashes_then_mouse_display);
    RUN_TEST(test_pairing_failure_three_red_flashes_then_back_to_pulse);
    RUN_TEST(test_flash_ordering_on_off_on);
    RUN_TEST(test_feedback_state_tracks_events_and_new_pairing_outcomes);
    RUN_TEST(test_light_pixel_clamps_intensity_at_the_source);
    RUN_TEST(test_frame_set_clamps_intensity);
    RUN_TEST(test_no_mode_ever_exceeds_the_ceiling);
    return UNITY_END();
}
