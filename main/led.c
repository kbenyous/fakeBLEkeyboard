#include "led.h"

typedef enum {
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_BLINK,
} led_mode_t;

static gpio_num_t s_gpio = GPIO_NUM_NC;
static bool s_active_low;

static led_mode_t s_mode = LED_MODE_OFF;
static uint32_t s_half_period_ms;
static uint32_t s_blink_ref_ms;   /* blink phase origin */
static bool s_restart_blink;      /* realign phase on next led_process() */

static uint32_t s_pulse_duration_ms;
static uint32_t s_pulse_start_ms;
static bool s_pulse_requested;
static bool s_pulse_active;

static bool s_level_on;           /* last physical state applied */

static void apply(bool on)
{
    if (s_gpio == GPIO_NUM_NC) {
        return;
    }
    s_level_on = on;
    gpio_set_level(s_gpio, (on ^ s_active_low) ? 1 : 0);
}

esp_err_t led_init(gpio_num_t gpio, bool active_low)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    s_gpio = gpio;
    s_active_low = active_low;
    apply(false);
    return ESP_OK;
}

void led_off(void)
{
    s_mode = LED_MODE_OFF;
}

void led_on(void)
{
    s_mode = LED_MODE_ON;
}

void led_blink(uint32_t period_ms)
{
    s_mode = LED_MODE_BLINK;
    s_half_period_ms = period_ms / 2 ? period_ms / 2 : 1;
    s_restart_blink = true;
}

void led_pulse(uint32_t duration_ms)
{
    s_pulse_duration_ms = duration_ms;
    s_pulse_requested = true;
}

static bool base_level(uint32_t now_ms)
{
    switch (s_mode) {
    case LED_MODE_ON:
        return true;
    case LED_MODE_BLINK:
        return ((now_ms - s_blink_ref_ms) / s_half_period_ms) % 2 == 0;
    case LED_MODE_OFF:
    default:
        return false;
    }
}

void led_process(uint32_t now_ms)
{
    if (s_restart_blink) {
        s_restart_blink = false;
        s_blink_ref_ms = now_ms;
    }
    if (s_pulse_requested) {
        s_pulse_requested = false;
        s_pulse_active = true;
        s_pulse_start_ms = now_ms;
    }
    if (s_pulse_active && now_ms - s_pulse_start_ms >= s_pulse_duration_ms) {
        s_pulse_active = false;
    }

    bool on = base_level(now_ms);
    if (s_pulse_active) {
        on = !on;
    }
    if (on != s_level_on) {
        apply(on);
    }
}
