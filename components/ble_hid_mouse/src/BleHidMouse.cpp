#include <BleHidMouse.hpp>

#include <cstring>
#include <esp_event.h>
#include <esp_hid_common.h>
#include <esp_hidd.h>
#include <esp_log.h>

#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

extern "C" void ble_store_config_init(void);

namespace Hilton {
    namespace {
        constexpr uint16_t HID_SERVICE_UUID16{0x1812};
        constexpr uint16_t CONN_HANDLE_NONE{0xFFFF};

        /**
         * Boot-compatible 3-button mouse with wheel, no report ID. Taken from
         * ESP-IDF examples/bluetooth/esp_hid_device (mouseReportMap, NimBLE mouse role).
         * Input report: [buttons(3 bits + 5 pad)] [X int8] [Y int8] [Wheel int8].
         */
        const uint8_t MOUSE_REPORT_MAP[] = {
            0x05, 0x01, // USAGE_PAGE (Generic Desktop)
            0x09, 0x02, // USAGE (Mouse)
            0xa1, 0x01, // COLLECTION (Application)
            0x09, 0x01, //   USAGE (Pointer)
            0xa1, 0x00, //   COLLECTION (Physical)
            0x05, 0x09, //     USAGE_PAGE (Button)
            0x19, 0x01, //     USAGE_MINIMUM (Button 1)
            0x29, 0x03, //     USAGE_MAXIMUM (Button 3)
            0x15, 0x00, //     LOGICAL_MINIMUM (0)
            0x25, 0x01, //     LOGICAL_MAXIMUM (1)
            0x95, 0x03, //     REPORT_COUNT (3)
            0x75, 0x01, //     REPORT_SIZE (1)
            0x81, 0x02, //     INPUT (Data,Var,Abs)
            0x95, 0x01, //     REPORT_COUNT (1)
            0x75, 0x05, //     REPORT_SIZE (5)
            0x81, 0x03, //     INPUT (Cnst,Var,Abs)
            0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
            0x09, 0x30, //     USAGE (X)
            0x09, 0x31, //     USAGE (Y)
            0x09, 0x38, //     USAGE (Wheel)
            0x15, 0x81, //     LOGICAL_MINIMUM (-127)
            0x25, 0x7f, //     LOGICAL_MAXIMUM (127)
            0x75, 0x08, //     REPORT_SIZE (8)
            0x95, 0x03, //     REPORT_COUNT (3)
            0x81, 0x06, //     INPUT (Data,Var,Rel)
            0xc0,       //   END_COLLECTION
            0xc0        // END_COLLECTION
        };

        esp_hid_raw_report_map_t s_reportMaps[] = {
            {.data = MOUSE_REPORT_MAP, .len = sizeof(MOUSE_REPORT_MAP)},
        };

        esp_hid_device_config_t s_hidConfig = {
            .vendor_id = 0x16C0, // Same VID/PID pair the IDF example uses (V-USB shared IDs)
            .product_id = 0x05DF,
            .version = 0x0100,
            .device_name = "BalanceMouse",
            .manufacturer_name = "Hilton",
            .serial_number = "0001",
            .report_maps = s_reportMaps,
            .report_maps_len = 1,
        };

        esp_hidd_dev_t *s_hidDev{nullptr};
        uint8_t s_ownAddrType{BLE_OWN_ADDR_PUBLIC};
        ble_hs_adv_fields s_advFields{};
        ble_uuid16_t s_hidUuid = BLE_UUID16_INIT(HID_SERVICE_UUID16);
        ble_npl_event s_forgetEvent{};
    } // namespace

    /** C-callback trampolines; all run on the NimBLE host task (or the esp_hidd event task). */
    struct BleHidMouseCallbacks {
        static int onGapEvent(ble_gap_event *event, void *arg);

        static void onHiddEvent(void *handlerArgs, esp_event_base_t base, int32_t id, void *eventData);

        static void onForgetBonds(ble_npl_event *ev);

        static void onSync();

        static void hostTask(void *param);
    };

    BleHidMouse &BleHidMouse::instance() {
        static BleHidMouse mouse;
        return mouse;
    }

    BleHidMouse::BleHidMouse()
        : reports([this](const MouseHidReport &r) { transmit(r); }), connHandle(CONN_HANDLE_NONE) {
    }

    PairingStatus BleHidMouse::getPairingStatus() const {
        PairingStatus s;
        s.sequence = pairingSequence.load();
        s.outcome = static_cast<PairingOutcome>(pairingOutcome.load());
        return s;
    }

    void BleHidMouse::recordPairing(const PairingOutcome outcome) {
        pairingOutcome.store(static_cast<uint8_t>(outcome));
        pairingSequence.fetch_add(1);
    }

    void BleHidMouse::transmit(const MouseHidReport &report) const {
        // Only send once the link is encrypted; esp_hidd would otherwise log an error per sample.
        if (phase.load() != ConnectionPhase::Connected || s_hidDev == nullptr) {
            return;
        }
        uint8_t buffer[4];
        std::memcpy(buffer, report.data(), sizeof(buffer));
        const esp_err_t err = esp_hidd_dev_input_set(s_hidDev, 0, 0, buffer, sizeof(buffer));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Input report failed: %s", esp_err_to_name(err));
        }
    }

    void BleHidMouse::startAdvertising() {
        if (connHandle.load() != CONN_HANDLE_NONE) {
            return;
        }
        int rc = ble_gap_adv_set_fields(&s_advFields);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
            return;
        }
        ble_gap_adv_params params{};
        params.conn_mode = BLE_GAP_CONN_MODE_UND;
        params.disc_mode = BLE_GAP_DISC_MODE_GEN;
        params.itvl_min = BLE_GAP_ADV_ITVL_MS(30); // HID recommendation: 30-50ms while (re)connecting
        params.itvl_max = BLE_GAP_ADV_ITVL_MS(50);
        rc = ble_gap_adv_start(s_ownAddrType, nullptr, BLE_HS_FOREVER, &params,
                               BleHidMouseCallbacks::onGapEvent, this);
        if (rc == BLE_HS_EALREADY) {
            rc = 0;
        }
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
            return;
        }
        phase.store(ConnectionPhase::Advertising);
        int bonds = 0;
        ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &bonds);
        ESP_LOGI(TAG, "Advertising as \"%s\" (%d bonded host%s)", name, bonds, bonds == 1 ? "" : "s");
    }

    int BleHidMouseCallbacks::onGapEvent(ble_gap_event *event, void *arg) {
        auto &self = *static_cast<BleHidMouse *>(arg);
        switch (event->type) {
            case BLE_GAP_EVENT_CONNECT:
                if (event->connect.status == 0) {
                    ESP_LOGI(BleHidMouse::TAG, "Connected (handle %d); requesting security", event->connect.conn_handle);
                    self.connHandle.store(event->connect.conn_handle);
                    self.phase.store(ConnectionPhase::Connecting);
                    // Bonded host: re-encrypts with the stored LTK. New host: starts pairing.
                    const int rc = ble_gap_security_initiate(event->connect.conn_handle);
                    if (rc != 0) {
                        ESP_LOGW(BleHidMouse::TAG, "ble_gap_security_initiate failed: %d", rc);
                    }
                } else {
                    ESP_LOGW(BleHidMouse::TAG, "Connection attempt failed: %d", event->connect.status);
                    self.connHandle.store(CONN_HANDLE_NONE);
                    self.startAdvertising();
                }
                return 0;

            case BLE_GAP_EVENT_DISCONNECT:
                ESP_LOGI(BleHidMouse::TAG, "Disconnected; reason=0x%x", event->disconnect.reason);
                self.connHandle.store(CONN_HANDLE_NONE);
                self.phase.store(ConnectionPhase::Advertising);
                if (self.clearBondsOnDisconnect.exchange(false)) {
                    const int rc = ble_store_clear();
                    ESP_LOGW(BleHidMouse::TAG, "Bonds erased after forced disconnect (rc=%d)", rc);
                }
                self.startAdvertising();
                return 0;

            case BLE_GAP_EVENT_ADV_COMPLETE:
                ESP_LOGI(BleHidMouse::TAG, "Advertising complete; reason=%d", event->adv_complete.reason);
                if (self.connHandle.load() == CONN_HANDLE_NONE) {
                    self.startAdvertising();
                }
                return 0;

            case BLE_GAP_EVENT_ENC_CHANGE:
                if (event->enc_change.status == 0) {
                    ESP_LOGI(BleHidMouse::TAG, "Link encrypted: pairing/re-connection succeeded");
                    self.phase.store(ConnectionPhase::Connected);
                    self.recordPairing(PairingOutcome::Success);
                } else {
                    ESP_LOGW(BleHidMouse::TAG, "Pairing/encryption failed; status=%d", event->enc_change.status);
                    self.recordPairing(PairingOutcome::Failed);
                    // Drop the half-open link so we go straight back to pairing mode.
                    ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
                return 0;

            case BLE_GAP_EVENT_REPEAT_PAIRING: {
                // The host lost its keys but we still have a bond: drop ours and pair again.
                ble_gap_conn_desc desc{};
                if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
                    ble_store_util_delete_peer(&desc.peer_id_addr);
                }
                return BLE_GAP_REPEAT_PAIRING_RETRY;
            }

            case BLE_GAP_EVENT_PASSKEY_ACTION: {
                // With NO_IO we should only ever see Just Works; accept a numeric comparison if a host insists.
                ble_sm_io pkey{};
                pkey.action = event->passkey.params.action;
                if (pkey.action == BLE_SM_IOACT_NUMCMP) {
                    pkey.numcmp_accept = 1;
                    ble_sm_inject_io(event->passkey.conn_handle, &pkey);
                }
                return 0;
            }

            case BLE_GAP_EVENT_SUBSCRIBE:
                ESP_LOGI(BleHidMouse::TAG, "Subscribe attr=%d notify=%d", event->subscribe.attr_handle,
                         event->subscribe.cur_notify);
                return 0;

            default:
                return 0;
        }
    }

    void BleHidMouseCallbacks::onHiddEvent(void *, esp_event_base_t, const int32_t id, void *) {
        switch (static_cast<esp_hidd_event_t>(id)) {
            case ESP_HIDD_START_EVENT:
                ESP_LOGI(BleHidMouse::TAG, "HID device started");
                BleHidMouse::instance().startAdvertising();
                break;
            case ESP_HIDD_CONNECT_EVENT:
                ESP_LOGI(BleHidMouse::TAG, "HID connect");
                break;
            case ESP_HIDD_DISCONNECT_EVENT:
                ESP_LOGI(BleHidMouse::TAG, "HID disconnect");
                break;
            default:
                break;
        }
    }

    void BleHidMouseCallbacks::onForgetBonds(ble_npl_event *) {
        auto &self = BleHidMouse::instance();
        const uint16_t conn = self.connHandle.load();
        self.recordPairing(PairingOutcome::None); // cancel any in-progress flash
        if (conn != CONN_HANDLE_NONE) {
            ESP_LOGW(BleHidMouse::TAG, "Forgetting bonds: disconnecting current host first");
            self.clearBondsOnDisconnect.store(true);
            self.phase.store(ConnectionPhase::Advertising);
            const int rc = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            if (rc != 0) {
                // Already gone: clear now and advertise.
                ESP_LOGW(BleHidMouse::TAG, "ble_gap_terminate failed (%d); clearing directly", rc);
                self.clearBondsOnDisconnect.store(false);
                self.connHandle.store(CONN_HANDLE_NONE);
                ble_store_clear();
                self.startAdvertising();
            }
            return;
        }
        ble_gap_adv_stop();
        const int rc = ble_store_clear();
        ESP_LOGW(BleHidMouse::TAG, "Bonds erased (rc=%d); back to pairing mode", rc);
        self.startAdvertising();
    }

    void BleHidMouseCallbacks::onSync() {
        // Overridden after esp_hidd_dev_init installed its own sync callback; chain to it via the HID START event.
        int rc = ble_hs_util_ensure_addr(0);
        if (rc != 0) {
            ESP_LOGE(BleHidMouse::TAG, "No usable BLE address: %d", rc);
        }
        rc = ble_hs_id_infer_auto(0, &s_ownAddrType);
        if (rc != 0) {
            ESP_LOGE(BleHidMouse::TAG, "ble_hs_id_infer_auto failed: %d", rc);
            s_ownAddrType = BLE_OWN_ADDR_PUBLIC;
        }
        BleHidMouse::instance().startAdvertising();
    }

    void BleHidMouseCallbacks::hostTask(void *) {
        ESP_LOGI(BleHidMouse::TAG, "NimBLE host task started");
        nimble_port_run(); // returns only after nimble_port_stop()
        nimble_port_freertos_deinit();
    }

    esp_err_t BleHidMouse::start(const char *deviceName) {
        if (deviceName != nullptr) {
            name = deviceName;
            s_hidConfig.device_name = deviceName;
        }

        esp_err_t err = nimble_port_init(); // also initialises the BT controller (IDF >= 5.0)
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
            return err;
        }

        // Security: Just Works (no display/keyboard on a balance board), bonding, LE Secure Connections.
        ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_mitm = 0;
        ble_hs_cfg.sm_sc = 1;
        ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
        ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

        // Advertising payload: flags, appearance (mouse), TX power, HID service UUID, name.
        s_advFields = {};
        s_advFields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
        s_advFields.appearance = ESP_HID_APPEARANCE_MOUSE;
        s_advFields.appearance_is_present = 1;
        s_advFields.tx_pwr_lvl_is_present = 1;
        s_advFields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
        s_advFields.name = reinterpret_cast<const uint8_t *>(name);
        s_advFields.name_len = static_cast<uint8_t>(std::strlen(name));
        s_advFields.name_is_complete = 1;
        s_advFields.uuids16 = &s_hidUuid;
        s_advFields.num_uuids16 = 1;
        s_advFields.uuids16_is_complete = 1;

        // HID-over-GATT service (registers GAP/GATT/DIS/BAS/HID services and its own GAP listener).
        err = esp_hidd_dev_init(&s_hidConfig, ESP_HID_TRANSPORT_BLE, BleHidMouseCallbacks::onHiddEvent, &s_hidDev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_hidd_dev_init failed: %s", esp_err_to_name(err));
            return err;
        }
        // esp_hidd_dev_init installed a sync callback that only posts ESP_HIDD_START_EVENT; we
        // want the address resolved before advertising, so take over sync and advertise from there.
        ble_hs_cfg.sync_cb = BleHidMouseCallbacks::onSync;

        ble_svc_gap_device_name_set(name);
        ble_svc_gap_device_appearance_set(ESP_HID_APPEARANCE_MOUSE);

        // Persist bonds in NVS (CONFIG_BT_NIMBLE_NVS_PERSIST) so a bonded host reconnects after reboot.
        ble_store_config_init();
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

        ble_npl_event_init(&s_forgetEvent, BleHidMouseCallbacks::onForgetBonds, nullptr);

        nimble_port_freertos_init(BleHidMouseCallbacks::hostTask);
        ESP_LOGI(TAG, "BLE HID mouse starting as \"%s\"", name);
        return ESP_OK;
    }

    void BleHidMouse::forgetBondsAndRestartPairing() {
        ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_forgetEvent);
    }
} // namespace Hilton
