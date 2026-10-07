#include "ble_hid_kbd.h"

#include <string.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Provided by NimBLE (store/config), no public header. */
void ble_store_config_init(void);

static const char *TAG = "ble_kbd";

#define APPEARANCE_HID_KEYBOARD 0x03C1

#define UUID_SVC_DEVICE_INFO    0x180A
#define UUID_SVC_BATTERY        0x180F
#define UUID_SVC_HID            0x1812
#define UUID_CHR_MANUFACTURER   0x2A29
#define UUID_CHR_PNP_ID         0x2A50
#define UUID_CHR_BATTERY_LEVEL  0x2A19
#define UUID_CHR_HID_INFO       0x2A4A
#define UUID_CHR_REPORT_MAP     0x2A4B
#define UUID_CHR_HID_CTRL_POINT 0x2A4C
#define UUID_CHR_REPORT         0x2A4D
#define UUID_DSC_REPORT_REF     0x2908

#define KBD_REPORT_ID      1
#define REPORT_TYPE_INPUT  1
#define REPORT_TYPE_OUTPUT 2

/* Standard keyboard: 8 modifiers, 1 reserved byte, 5 LEDs (output), 6 simultaneous keys. */
static const uint8_t s_report_map[] = {
    0x05, 0x01,        /* Usage Page (Generic Desktop) */
    0x09, 0x06,        /* Usage (Keyboard) */
    0xA1, 0x01,        /* Collection (Application) */
    0x85, KBD_REPORT_ID, /*   Report ID */
    0x05, 0x07,        /*   Usage Page (Keyboard/Keypad) */
    0x19, 0xE0,        /*   Usage Minimum (Left Control) */
    0x29, 0xE7,        /*   Usage Maximum (Right GUI) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x01,        /*   Logical Maximum (1) */
    0x75, 0x01,        /*   Report Size (1) */
    0x95, 0x08,        /*   Report Count (8) */
    0x81, 0x02,        /*   Input (Data, Var, Abs): modifiers */
    0x95, 0x01,        /*   Report Count (1) */
    0x75, 0x08,        /*   Report Size (8) */
    0x81, 0x01,        /*   Input (Const): reserved */
    0x95, 0x05,        /*   Report Count (5) */
    0x75, 0x01,        /*   Report Size (1) */
    0x05, 0x08,        /*   Usage Page (LEDs) */
    0x19, 0x01,        /*   Usage Minimum (Num Lock) */
    0x29, 0x05,        /*   Usage Maximum (Kana) */
    0x91, 0x02,        /*   Output (Data, Var, Abs): LEDs */
    0x95, 0x01,        /*   Report Count (1) */
    0x75, 0x03,        /*   Report Size (3) */
    0x91, 0x01,        /*   Output (Const): padding */
    0x95, 0x06,        /*   Report Count (6) */
    0x75, 0x08,        /*   Report Size (8) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x73,        /*   Logical Maximum (0x73 = F24) */
    0x05, 0x07,        /*   Usage Page (Keyboard/Keypad) */
    0x19, 0x00,        /*   Usage Minimum (0) */
    0x29, 0x73,        /*   Usage Maximum (F24) */
    0x81, 0x00,        /*   Input (Data, Array, Abs): keys */
    0xC0,              /* End Collection */
};

/* bcdHID 1.11, country 0, flags: RemoteWake | NormallyConnectable */
static const uint8_t s_hid_info[4] = {0x11, 0x01, 0x00, 0x03};
/* PnP ID: USB source (0x02), VID 0x303A (Espressif), PID 0x4001, version 0x0100 */
static const uint8_t s_pnp_id[7] = {0x02, 0x3A, 0x30, 0x01, 0x40, 0x00, 0x01};
static const uint8_t s_input_report_ref[2] = {KBD_REPORT_ID, REPORT_TYPE_INPUT};
static const uint8_t s_output_report_ref[2] = {KBD_REPORT_ID, REPORT_TYPE_OUTPUT};
static const uint8_t s_battery_level = 100;

static uint8_t s_input_report[8];
static uint8_t s_output_report; /* keyboard LED state sent by the host (ignored) */
static uint16_t s_input_report_handle;

static ble_kbd_config_t s_cfg;
static uint8_t s_own_addr_type;

/*
 * Connection state (written by the NimBLE task, read by the application task).
 * NimBLE defers a peripheral's CONNECT event until the remote version is read:
 * ENC_CHANGE and SUBSCRIBE may therefore arrive before it. The connection is
 * registered on the first event that mentions it, and the state is only reset
 * on disconnection.
 */
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_encrypted;
static volatile bool s_input_notify;
static volatile bool s_host_ready;
static volatile bool s_pairing_window;
/* Number of known legitimate bonds: a higher count signals a new pairing. */
static volatile int s_known_bonds;

typedef enum {
    ATTR_MANUFACTURER,
    ATTR_PNP_ID,
    ATTR_BATTERY_LEVEL,
    ATTR_HID_INFO,
    ATTR_REPORT_MAP,
    ATTR_HID_CTRL_POINT,
    ATTR_INPUT_REPORT,
    ATTR_INPUT_REPORT_REF,
    ATTR_OUTPUT_REPORT,
    ATTR_OUTPUT_REPORT_REF,
} attr_id_t;

static void emit(ble_kbd_event_t event)
{
    if (s_cfg.on_event) {
        s_cfg.on_event(event, s_cfg.ctx);
    }
}

static int bond_count(void)
{
    int count = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count);
    return count;
}

/* ---------- GATT ---------- */

static int read_value(struct ble_gatt_access_ctxt *ctxt, const void *data, uint16_t len)
{
    return os_mbuf_append(ctxt->om, data, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch ((attr_id_t)(uintptr_t)arg) {
    case ATTR_MANUFACTURER:
        return read_value(ctxt, s_cfg.manufacturer, strlen(s_cfg.manufacturer));
    case ATTR_PNP_ID:
        return read_value(ctxt, s_pnp_id, sizeof(s_pnp_id));
    case ATTR_BATTERY_LEVEL:
        return read_value(ctxt, &s_battery_level, sizeof(s_battery_level));
    case ATTR_HID_INFO:
        return read_value(ctxt, s_hid_info, sizeof(s_hid_info));
    case ATTR_REPORT_MAP:
        return read_value(ctxt, s_report_map, sizeof(s_report_map));
    case ATTR_HID_CTRL_POINT:
        /* Suspend / Exit Suspend: nothing to do. */
        return 0;
    case ATTR_INPUT_REPORT:
        return read_value(ctxt, s_input_report, sizeof(s_input_report));
    case ATTR_INPUT_REPORT_REF:
        return read_value(ctxt, s_input_report_ref, sizeof(s_input_report_ref));
    case ATTR_OUTPUT_REPORT:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            if (OS_MBUF_PKTLEN(ctxt->om) != sizeof(s_output_report)) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            ble_hs_mbuf_to_flat(ctxt->om, &s_output_report, sizeof(s_output_report), NULL);
            ESP_LOGD(TAG, "Keyboard LEDs: 0x%02x", s_output_report);
            return 0;
        }
        return read_value(ctxt, &s_output_report, sizeof(s_output_report));
    case ATTR_OUTPUT_REPORT_REF:
        return read_value(ctxt, s_output_report_ref, sizeof(s_output_report_ref));
    }
    return BLE_ATT_ERR_UNLIKELY;
}

#define ATTR(id) ((void *)(uintptr_t)(id))

static const struct ble_gatt_svc_def s_gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_SVC_DEVICE_INFO),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_MANUFACTURER),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_MANUFACTURER),
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_PNP_ID),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_PNP_ID),
                .flags = BLE_GATT_CHR_F_READ,
            },
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_SVC_BATTERY),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_BATTERY_LEVEL),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_BATTERY_LEVEL),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_SVC_HID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_HID_INFO),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_HID_INFO),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
            },
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_REPORT_MAP),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_REPORT_MAP),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
            },
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_HID_CTRL_POINT),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_HID_CTRL_POINT),
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
            },
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_REPORT),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_INPUT_REPORT),
                .val_handle = &s_input_report_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(UUID_DSC_REPORT_REF),
                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
                        .access_cb = gatt_access,
                        .arg = ATTR(ATTR_INPUT_REPORT_REF),
                    },
                    {0},
                },
            },
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_REPORT),
                .access_cb = gatt_access,
                .arg = ATTR(ATTR_OUTPUT_REPORT),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC |
                         BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC |
                         BLE_GATT_CHR_F_WRITE_NO_RSP,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(UUID_DSC_REPORT_REF),
                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
                        .access_cb = gatt_access,
                        .arg = ATTR(ATTR_OUTPUT_REPORT_REF),
                    },
                    {0},
                },
            },
            {0},
        },
    },
    {0},
};

/* ---------- GAP ---------- */

/* Registers the connection on the first event that mentions it, whatever it is. */
static void track_connection(uint16_t conn_handle)
{
    if (s_conn_handle == conn_handle) {
        return;
    }
    ESP_LOGI(TAG, "Connected (handle=%d)", conn_handle);
    s_conn_handle = conn_handle;
    emit(BLE_KBD_EVT_CONNECTED);
}

static void reset_connection_state(void)
{
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_encrypted = false;
    s_input_notify = false;
    s_host_ready = false;
}

static void update_host_ready(void)
{
    bool ready = s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_encrypted && s_input_notify;
    if (ready && !s_host_ready) {
        s_host_ready = true;
        ESP_LOGI(TAG, "Host ready to receive keys");
        emit(BLE_KBD_EVT_HOST_READY);
    }
}

static void on_connect(uint16_t conn_handle, int status)
{
    if (status != 0) {
        /* If the link still exists, DISCONNECT will follow and the app will restart advertising. */
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle, &desc) != 0 && s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGW(TAG, "Connection failed (status=%d), resuming advertising", status);
            ble_kbd_advertise(s_pairing_window);
        }
        return;
    }

    track_connection(conn_handle);

    /* Request security from the peripheral side, unless the host already encrypted the link. */
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) == 0 && !desc.sec_state.encrypted) {
        ble_gap_security_initiate(conn_handle);
    }
}

static void on_encryption_change(uint16_t conn_handle, int status)
{
    track_connection(conn_handle);

    if (status != 0) {
        ESP_LOGW(TAG, "Link security failed (status=%d)", status);
        emit(BLE_KBD_EVT_SECURITY_FAILED);
        ble_gap_terminate(conn_handle, BLE_ERR_AUTH_FAIL);
        return;
    }

    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) != 0) {
        return;
    }
    if (!desc.sec_state.bonded) {
        ESP_LOGW(TAG, "Link encrypted without bonding: rejected");
        emit(BLE_KBD_EVT_SECURITY_FAILED);
        ble_gap_terminate(conn_handle, BLE_ERR_AUTH_FAIL);
        return;
    }

    /*
     * NimBLE persists the keys before emitting ENC_CHANGE: more bonds than the known
     * legitimate ones = new pairing. A mere encryption restore (reconnection of a
     * known host) does not change the count.
     */
    int bonds = bond_count();
    if (bonds > s_known_bonds) {
        if (!s_pairing_window) {
            ESP_LOGW(TAG, "Pairing outside the pairing window: rejected");
            ble_store_util_delete_peer(&desc.peer_id_addr);
            s_known_bonds = bond_count();
            emit(BLE_KBD_EVT_SECURITY_FAILED);
            ble_gap_terminate(conn_handle, BLE_ERR_AUTH_FAIL);
            return;
        }
        s_pairing_window = false;
        s_known_bonds = bonds;
        ESP_LOGI(TAG, "New host paired (%d bond(s) in memory)", bonds);
        emit(BLE_KBD_EVT_PAIRED);
    }

    s_encrypted = true;
    update_host_ready();
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        on_connect(event->connect.conn_handle, event->connect.status);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected (reason=0x%x)", event->disconnect.reason);
        reset_connection_state();
        emit(BLE_KBD_EVT_DISCONNECTED);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        on_encryption_change(event->enc_change.conn_handle, event->enc_change.status);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_input_report_handle) {
            track_connection(event->subscribe.conn_handle);
            s_input_notify = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "Keyboard report notifications: %s", s_input_notify ? "on" : "off");
            update_host_ready();
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* Already-bonded host wanting to re-pair (it lost its keys). */
        if (!s_pairing_window) {
            ESP_LOGW(TAG, "Re-pairing outside the pairing window: ignored");
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
            s_known_bonds = bond_count();
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
    case BLE_GAP_EVENT_MTU:
    case BLE_GAP_EVENT_ADV_COMPLETE:
    default:
        return 0;
    }
}

/* ---------- Stack ---------- */

static void on_stack_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE stack reset (reason=%d)", reason);
}

static void on_stack_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not determine BLE address (rc=%d)", rc);
        return;
    }
    s_known_bonds = bond_count();
    ESP_LOGI(TAG, "Stack ready, %d bond(s) in memory", s_known_bonds);
    emit(BLE_KBD_EVT_STACK_READY);
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_kbd_init(const ble_kbd_config_t *config)
{
    s_cfg = *config;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* "Just Works" pairing, Secure Connections, bonding with identity key exchange. */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_services);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_gatt_services);
    }
    if (rc == 0) {
        rc = ble_svc_gap_device_name_set(s_cfg.device_name);
    }
    if (rc == 0) {
        rc = ble_svc_gap_device_appearance_set(APPEARANCE_HID_KEYBOARD);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT/GAP configuration failed (rc=%d)", rc);
        return ESP_FAIL;
    }

    ble_store_config_init();
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

/* ---------- API ---------- */

bool ble_kbd_has_bond(void)
{
    return bond_count() > 0;
}

esp_err_t ble_kbd_erase_bonds(void)
{
    int rc = ble_store_clear();
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not erase bonds (rc=%d)", rc);
        return ESP_FAIL;
    }
    s_known_bonds = 0;
    ESP_LOGI(TAG, "Bonds erased");
    return ESP_OK;
}

esp_err_t ble_kbd_advertise(bool accept_new_pairing)
{
    s_pairing_window = accept_new_pairing;

    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance = APPEARANCE_HID_KEYBOARD;
    fields.appearance_is_present = 1;
    fields.uuids16 = (ble_uuid16_t[]) {BLE_UUID16_INIT(UUID_SVC_HID)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.name = (const uint8_t *)s_cfg.device_name;
    fields.name_len = strlen(s_cfg.device_name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Invalid advertising data (rc=%d)", rc);
        return ESP_FAIL;
    }

    const struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = BLE_GAP_ADV_ITVL_MS(30),
        .itvl_max = BLE_GAP_ADV_ITVL_MS(50),
    };
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not start advertising (rc=%d)", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Advertising started (%s)", accept_new_pairing ? "pairing" : "reconnection");
    return ESP_OK;
}

esp_err_t ble_kbd_stop_advertising(void)
{
    s_pairing_window = false;
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }
    return ESP_OK;
}

esp_err_t ble_kbd_disconnect(void)
{
    uint16_t conn = s_conn_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    return ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM) == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t send_input_report(uint16_t conn)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(s_input_report, sizeof(s_input_report));
    if (om == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* ble_gatts_notify_custom frees om in all cases. */
    return ble_gatts_notify_custom(conn, s_input_report_handle, om) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t ble_kbd_tap_key(uint8_t keycode)
{
    uint16_t conn = s_conn_handle;
    if (!s_host_ready || conn == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(s_input_report, 0, sizeof(s_input_report));
    s_input_report[2] = keycode;
    esp_err_t err = send_input_report(conn);

    /* Release sent even if the press failed, to never leave a key held down. */
    memset(s_input_report, 0, sizeof(s_input_report));
    esp_err_t release_err = send_input_report(conn);

    return err != ESP_OK ? err : release_err;
}
