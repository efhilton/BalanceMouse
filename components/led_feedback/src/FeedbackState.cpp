#include <FeedbackState.hpp>
#include <cstdlib>

namespace Hilton {
    int FeedbackState::indexOf(const Button button) {
        switch (button) {
            case Button::Left:
                return 0;
            case Button::Right:
                return 1;
            case Button::Middle:
                return 2;
        }
        return 0;
    }

    void FeedbackState::onButtonDown(const Button button) {
        const uint32_t now = clock();
        std::lock_guard lock(mutex);
        const int i = indexOf(button);
        state.buttonPressed[i] = true;
        state.buttonPressedAtMs[i] = now;
    }

    void FeedbackState::onButtonUp(const Button button) {
        std::lock_guard lock(mutex);
        state.buttonPressed[indexOf(button)] = false;
    }

    void FeedbackState::onMovement(int8_t, int8_t) {
        // The ball is driven by calibrated tilt (updateMotion), not by deltas.
    }

    void FeedbackState::onScroll(const int8_t ticks) {
        if (ticks == 0) {
            return;
        }
        const uint32_t now = clock();
        std::lock_guard lock(mutex);
        // ticks share calibratedYaw's sign (GestureDetector::handleScrollWheel), same as
        // `angle` in updateMotion() below -- so this must use the identical sign flip, or the
        // brief tick-hold window (see snapshot()) would show the opposite chevron direction
        // from the live-angle path for the same physical gesture. See updateMotion() for why.
        lastTickDirection = ticks > 0 ? -1 : 1;
        lastTickMs = now;
        hasTick = true;
    }

    void FeedbackState::updateMotion(const float tiltX, const float tiltY, const int8_t angle) {
        std::lock_guard lock(mutex);
        // Ball display spec (#1181, Edgar): spring/damper from matrix centre, right-hand-rule
        // axes (X=right, Y=forward, Z=up), with the top/left/right button bars confirmed at
        // +Y/-X/+X:
        //   tilt -X -> ball +Y      tilt +X -> ball -Y
        //   tilt +Y -> ball +X      tilt -Y -> ball -X
        //
        // FINAL, hardware-confirmed mapping: X passes through unchanged, Y is negated.
        // This was found empirically over three rounds of real-hardware testing by Edgar
        // (2026-11-02), NOT derived from physics (an earlier physics derivation predicted the
        // wrong signs and was dropped). Round 3, on a build with no correction on either axis:
        // "+/- X direction is correct now", but Y was backwards (tilting forward moved the ball
        // backwards) -- hence the Y-only negation. ballPosition() maps "bigger value -> bigger
        // pixel index" (pixel +x = right, pixel +y = down).
        //
        // If a display-only sign correction is ever needed again, it belongs HERE, not in
        // ballPosition(): that is a pure, axis-symmetric mapping shared by both axes (its own
        // tests rely on that), and FeedbackSnapshot::tiltX/Y are documented as display-space.
        // Likewise keep any such correction scoped to this display path -- GestureDetector's HID
        // movement (handleMouseMovement) consumes the same calibratedTiltX/Y independently, and
        // its sign is still unverified on real hardware (HID transport stays disabled, #1181).
        state.tiltX = tiltX;
        state.tiltY = -tiltY;
        scrollAngle = angle;
        if (angle != 0) {
            // Held across the brief tick-hold window in snapshot() so chevron speed doesn't
            // dip to "slowest" every time `angle` happens to read 0 between samples.
            lastMagnitude = static_cast<uint8_t>(std::abs(static_cast<int>(angle)));
        }
    }

    void FeedbackState::updateConnection(const ConnectionPhase phase, const PairingStatus &pairing) {
        const uint32_t now = clock();
        std::lock_guard lock(mutex);
        state.phase = phase;
        if (pairing.sequence != lastPairingSequence) {
            lastPairingSequence = pairing.sequence;
            state.pairingOutcome = pairing.outcome;
            state.pairingOutcomeAtMs = now;
        }
    }

    FeedbackSnapshot FeedbackState::snapshot() const {
        const uint32_t now = clock();
        std::lock_guard lock(mutex);
        FeedbackSnapshot s = state;
        if (scrollAngle != 0) {
            // #1181, Edgar's explicit spec: physical +Z rotation (positive calibratedYaw,
            // which is what `angle` is scaled from -- see GestureDetector::handleScrollWheel)
            // must show the scroll-DOWN chevron pattern, -Z shows scroll-UP. That pattern is
            // what this file (and FeedbackRenderer/renderChevrons) calls direction -1 and +1
            // respectively, hence the flip here rather than the naive angle>0 -> +1 mapping.
            // Does not touch the actual HID wheel tick value/sign anywhere (GestureDetector,
            // Mouse) -- this is a display-only mapping, scoped to exactly what was asked:
            // verify the gesture via the chevron, independent of the still-disabled HID output.
            s.scrollDirection = scrollAngle > 0 ? -1 : 1;
            s.scrollMagnitude = static_cast<uint8_t>(std::abs(static_cast<int>(scrollAngle)));
        } else if (hasTick && (now - lastTickMs) < SCROLL_HOLD_MS) {
            s.scrollDirection = lastTickDirection;
            s.scrollMagnitude = lastMagnitude;
        } else {
            s.scrollDirection = 0;
            s.scrollMagnitude = 0;
        }
        return s;
    }
} // namespace Hilton
