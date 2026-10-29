#pragma once

#include <ConnectionState.hpp>
#include <MouseHidReport.hpp>
#include <atomic>
#include <esp_err.h>

namespace Hilton {
    /**
     * BLE HID-over-GATT mouse transport (NimBLE host + esp_hid's esp_hidd device API).
     *
     *  - Advertises (undirected, connectable, general-discoverable) whenever no host
     *    is connected: that is "pairing mode" when unbonded, and is also how an
     *    already-bonded host reconnects automatically after boot (bonds persist in NVS).
     *  - "Just Works" pairing (no IO), bonding + LE Secure Connections.
     *  - Implements MouseEventSink: every button down/up, movement delta and scroll
     *    tick becomes one 4-byte HID input report (see MouseHidReport.hpp), sent only
     *    while connected and encrypted.
     *  - Exposes the connection phase and the latest pairing outcome for the LEDs.
     *
     * Singleton because NimBLE / esp_hidd callbacks are plain C function pointers.
     */
    class BleHidMouse final : public MouseEventSink {
    public:
        static constexpr auto TAG{"BLE_HID_MOUSE"};

        static BleHidMouse &instance();

        /** Requires nvs_flash_init() to have been called. Starts the NimBLE host task. */
        esp_err_t start(const char *deviceName);

        /**
         * Forget every bond and go back to pairing mode. Safe to call from any task
         * (the work is posted to the NimBLE host task). If a host is connected it is
         * disconnected first, then the bond store is erased and advertising restarts.
         */
        void forgetBondsAndRestartPairing();

        [[nodiscard]] ConnectionPhase getPhase() const { return phase.load(); }

        [[nodiscard]] PairingStatus getPairingStatus() const;

        // MouseEventSink
        void onButtonDown(Button button) override { reports.onButtonDown(button); }
        void onButtonUp(Button button) override { reports.onButtonUp(button); }
        void onMovement(int8_t dx, int8_t dy) override { reports.onMovement(dx, dy); }
        void onScroll(int8_t ticks) override { reports.onScroll(ticks); }

        BleHidMouse(const BleHidMouse &) = delete;

        BleHidMouse &operator=(const BleHidMouse &) = delete;

    private:
        BleHidMouse();

        void transmit(const MouseHidReport &report) const;

        // --- Called on the NimBLE host task only ---
        friend struct BleHidMouseCallbacks;

        void startAdvertising();

        void recordPairing(PairingOutcome outcome);

        MouseHidReportBuilder reports;
        std::atomic<ConnectionPhase> phase{ConnectionPhase::Idle};
        std::atomic<uint8_t> pairingOutcome{static_cast<uint8_t>(PairingOutcome::None)};
        std::atomic<uint32_t> pairingSequence{0};
        std::atomic<uint16_t> connHandle;
        std::atomic<bool> clearBondsOnDisconnect{false};
        const char *name{"BalanceMouse"};
    };
} // namespace Hilton
