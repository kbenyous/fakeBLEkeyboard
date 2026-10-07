/*
 * HID keyboard over GATT (HOGP) on NimBLE.
 *
 * The module manages the BLE stack, the HID service, security ("Just Works" pairing
 * with bonding persisted in NVS) and key sending. It holds no application logic:
 * it reports events via a callback, and the application decides when to advertise,
 * pair or send a key.
 *
 * WARNING: the callback runs in the NimBLE host task. It must be short and
 * non-blocking (typically: post a message into a queue).
 *
 * Prerequisite: NVS initialized (nvs_flash_init) before ble_kbd_init().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HID usage codes (Keyboard/Keypad page 0x07) used here. */
#define HID_KEY_F13 0x68
#define HID_KEY_F14 0x69
#define HID_KEY_F15 0x6A

typedef enum {
    BLE_KBD_EVT_STACK_READY,     /* stack synced: advertising is possible */
    BLE_KBD_EVT_CONNECTED,       /* link established (not secured yet) */
    BLE_KBD_EVT_PAIRED,          /* new bond stored during the pairing window */
    BLE_KBD_EVT_SECURITY_FAILED, /* pairing/encryption failure, or pairing rejected */
    BLE_KBD_EVT_HOST_READY,      /* link encrypted + bonded + notifications on: ready to type */
    BLE_KBD_EVT_DISCONNECTED,
} ble_kbd_event_t;

typedef void (*ble_kbd_event_cb_t)(ble_kbd_event_t event, void *ctx);

typedef struct {
    const char *device_name;   /* advertised name; <= 18 characters to fit in the advertisement */
    const char *manufacturer;
    ble_kbd_event_cb_t on_event;
    void *ctx;
} ble_kbd_config_t;

esp_err_t ble_kbd_init(const ble_kbd_config_t *config);

/* True if at least one host is paired (persisted bond). */
bool ble_kbd_has_bond(void);

/* Erases all persisted bonds. */
esp_err_t ble_kbd_erase_bonds(void);

/*
 * Starts (or restarts) connectable advertising.
 * accept_new_pairing = true opens the pairing window: a new host can bond.
 * It closes automatically on the first successful pairing.
 * Outside the window, any pairing attempt from an unknown host is rejected.
 */
esp_err_t ble_kbd_advertise(bool accept_new_pairing);

esp_err_t ble_kbd_stop_advertising(void);

/* Drops the current connection (DISCONNECTED event to follow). */
esp_err_t ble_kbd_disconnect(void);

/* Press + release a key. ESP_ERR_INVALID_STATE if the host is not ready. */
esp_err_t ble_kbd_tap_key(uint8_t keycode);

#ifdef __cplusplus
}
#endif
