/*
 * Application: fake BLE keyboard that presses F15 at a pseudo-random interval.
 *
 * All application logic lives here. The modules (BLE, LED, button) do not know about
 * each other: they report events via callback, which the application posts into a
 * single queue, handled by the one event loop below. That loop also clocks the
 * task-less modules (LED, button) and the key timer.
 *
 * State machine:
 *   STARTING      BLE stack not ready yet
 *   IDLE          no host paired, nothing advertised              LED off
 *   PAIRING       pairing window open                             LED blinks at 4 Hz
 *   RECONNECTING  host paired, waiting for (re)connection         LED off
 *   CONNECTED     host connected and ready: F15 every 15 to 30 s  LED steady (pulse on each key)
 *
 * Pressing BOOT, in any state, erases the pairing and (re)starts pairing.
 */
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "ble_hid_kbd.h"
#include "button.h"
#include "led.h"

static const char *TAG = "app";

/* ESP32-C3 SuperMini: blue LED on GPIO8 (active low), BOOT button on GPIO9. */
#define LED_GPIO          GPIO_NUM_8
#define LED_ACTIVE_LOW    true
#define BUTTON_GPIO       GPIO_NUM_9
#define BUTTON_ACTIVE_LOW true

#define DEVICE_NAME       "BLE Keyboard"
#define MANUFACTURER      "DIY"

#define PAIRING_BLINK_PERIOD_MS 250  /* 4 blinks per second */
#define KEY_PULSE_MS            120
#define KEY_INTERVAL_MIN_MS     15000
#define KEY_INTERVAL_MAX_MS     30000
#define KEY_CODE                HID_KEY_F15

#define LOOP_TICK_MS      10
#define EVENT_QUEUE_LEN   16

typedef enum {
    APP_STATE_STARTING,
    APP_STATE_IDLE,
    APP_STATE_PAIRING,
    APP_STATE_RECONNECTING,
    APP_STATE_CONNECTED,
} app_state_t;

typedef enum {
    APP_EVT_BUTTON_PRESSED,
    APP_EVT_BLE,
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    ble_kbd_event_t ble;
} app_event_t;

static QueueHandle_t s_events;
static app_state_t s_state = APP_STATE_STARTING;
static bool s_connected;
static uint32_t s_next_key_ms;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ---------- Module callbacks: only post into the queue ---------- */

static void post_event(const app_event_t *event)
{
    if (xQueueSend(s_events, event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Event queue full, event dropped (type=%d)", event->type);
    }
}

static void on_button_pressed(void *ctx)
{
    post_event(&(app_event_t) {.type = APP_EVT_BUTTON_PRESSED});
}

static void on_ble_event(ble_kbd_event_t event, void *ctx)
{
    post_event(&(app_event_t) {.type = APP_EVT_BLE, .ble = event});
}

/* ---------- State transitions ---------- */

static void enter_idle(void)
{
    ESP_LOGI(TAG, "State: IDLE (no host paired, press BOOT to pair)");
    s_state = APP_STATE_IDLE;
    led_off();
    ble_kbd_stop_advertising();
}

static void enter_reconnecting(void)
{
    ESP_LOGI(TAG, "State: RECONNECTING");
    s_state = APP_STATE_RECONNECTING;
    led_off();
    if (!s_connected) {
        ble_kbd_advertise(false);
    }
}

static void enter_pairing(void)
{
    ESP_LOGI(TAG, "State: PAIRING");
    s_state = APP_STATE_PAIRING;
    led_blink(PAIRING_BLINK_PERIOD_MS);
    ble_kbd_erase_bonds();
    if (s_connected) {
        /* Advertising will restart when DISCONNECTED is received. */
        ble_kbd_disconnect();
    } else {
        ble_kbd_advertise(true);
    }
}

static void schedule_next_key(uint32_t now)
{
    uint32_t span = KEY_INTERVAL_MAX_MS - KEY_INTERVAL_MIN_MS + 1;
    uint32_t delay = KEY_INTERVAL_MIN_MS + esp_random() % span;
    s_next_key_ms = now + delay;
    ESP_LOGI(TAG, "Next key in %" PRIu32 " ms", delay);
}

static void enter_connected(void)
{
    ESP_LOGI(TAG, "State: CONNECTED");
    s_state = APP_STATE_CONNECTED;
    led_on();
    schedule_next_key(now_ms());
}

/* ---------- Event handling ---------- */

static void handle_ble_event(ble_kbd_event_t event)
{
    switch (event) {
    case BLE_KBD_EVT_STACK_READY:
        if (s_state == APP_STATE_PAIRING) {
            ble_kbd_advertise(true);
        } else if (ble_kbd_has_bond()) {
            enter_reconnecting();
        } else {
            enter_idle();
        }
        break;

    case BLE_KBD_EVT_CONNECTED:
        s_connected = true;
        break;

    case BLE_KBD_EVT_PAIRED:
        /* Pairing window is closed: on a drop, we wait for reconnection. */
        ESP_LOGI(TAG, "Pairing successful");
        s_state = APP_STATE_RECONNECTING;
        break;

    case BLE_KBD_EVT_SECURITY_FAILED:
        ESP_LOGW(TAG, "Security failed, connection will be closed");
        break;

    case BLE_KBD_EVT_HOST_READY:
        enter_connected();
        break;

    case BLE_KBD_EVT_DISCONNECTED:
        s_connected = false;
        if (s_state == APP_STATE_PAIRING) {
            ble_kbd_advertise(true);
        } else if (ble_kbd_has_bond()) {
            enter_reconnecting();
        } else {
            enter_idle();
        }
        break;
    }
}

static void handle_event(const app_event_t *event)
{
    switch (event->type) {
    case APP_EVT_BUTTON_PRESSED:
        if (s_state == APP_STATE_STARTING) {
            ESP_LOGW(TAG, "BLE stack not ready yet, press ignored");
            break;
        }
        ESP_LOGI(TAG, "Button: new pairing cycle");
        enter_pairing();
        break;

    case APP_EVT_BLE:
        handle_ble_event(event->ble);
        break;
    }
}

static void process_key_timer(uint32_t now)
{
    if (s_state != APP_STATE_CONNECTED || (int32_t)(now - s_next_key_ms) < 0) {
        return;
    }
    esp_err_t err = ble_kbd_tap_key(KEY_CODE);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "F15 key sent");
        led_pulse(KEY_PULSE_MS);
    } else {
        ESP_LOGW(TAG, "Could not send key: %s", esp_err_to_name(err));
    }
    schedule_next_key(now);
}

/* ---------- Entry point ---------- */

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS unusable (%s): erasing, pairings are lost", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}


void app_main(void)
{
    init_nvs();

    s_events = xQueueCreate(EVENT_QUEUE_LEN, sizeof(app_event_t));
    configASSERT(s_events);

    ESP_ERROR_CHECK(led_init(LED_GPIO, LED_ACTIVE_LOW));
    ESP_ERROR_CHECK(button_init(BUTTON_GPIO, BUTTON_ACTIVE_LOW, on_button_pressed, NULL));

    const ble_kbd_config_t ble_config = {
        .device_name = DEVICE_NAME,
        .manufacturer = MANUFACTURER,
        .on_event = on_ble_event,
        .ctx = NULL,
    };
    ESP_ERROR_CHECK(ble_kbd_init(&ble_config));

    /* The application's single event loop. */
    for (;;) {
        app_event_t event;
        if (xQueueReceive(s_events, &event, pdMS_TO_TICKS(LOOP_TICK_MS)) == pdTRUE) {
            handle_event(&event);
        }

        uint32_t now = now_ms();
        button_process(now);
        led_process(now);
        process_key_timer(now);
    }
}
