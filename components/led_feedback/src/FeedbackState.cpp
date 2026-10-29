#include <FeedbackState.hpp>

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
        lastTickDirection = ticks > 0 ? 1 : -1;
        lastTickMs = now;
        hasTick = true;
    }

    void FeedbackState::updateMotion(const float tiltX, const float tiltY, const int8_t angle) {
        std::lock_guard lock(mutex);
        state.tiltX = tiltX;
        state.tiltY = tiltY;
        scrollAngle = angle;
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
            s.scrollDirection = scrollAngle > 0 ? 1 : -1;
        } else if (hasTick && (now - lastTickMs) < SCROLL_HOLD_MS) {
            s.scrollDirection = lastTickDirection;
        } else {
            s.scrollDirection = 0;
        }
        return s;
    }
} // namespace Hilton
