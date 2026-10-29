#pragma once

#include <cstdint>

namespace Hilton {
    /** Transport connection phase, as exposed to the LED feedback layer. */
    enum class ConnectionPhase : uint8_t {
        /** Transport not started yet / stopped. Rendered like Advertising (waiting for a host). */
        Idle,
        /** Advertising and waiting for a host: either unbonded "pairing mode" or a bonded host reconnecting. */
        Advertising,
        /** A host has connected; the link may still be (re-)encrypting/pairing. */
        Connecting,
        /** Connected and encrypted; HID reports are being delivered. */
        Connected,
    };

    enum class PairingOutcome : uint8_t {
        None,
        Success,
        Failed,
    };

    /**
     * Most recent pairing/encryption outcome. `sequence` increments every time
     * a new outcome is recorded so pollers can detect a fresh event even when
     * the outcome value repeats (e.g. two failures in a row).
     */
    struct PairingStatus {
        PairingOutcome outcome{PairingOutcome::None};
        uint32_t sequence{0};
    };
} // namespace Hilton
