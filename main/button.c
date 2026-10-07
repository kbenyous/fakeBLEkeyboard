#include "button.h"

#define DEBOUNCE_MS 30

static gpio_num_t s_gpio = GPIO_NUM_NC;
static bool s_active_low;
static button_press_cb_t s_on_press;
static void *s_ctx;

static bool s_stable_pressed;
static bool s_last_raw;
static uint32_t s_last_change_ms;

static bool read_pressed(void)
{
    return (gpio_get_level(s_gpio) != 0) ^ s_active_low;
}

esp_err_t button_init(gpio_num_t gpio, bool active_low, button_press_cb_t on_press, void *ctx)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    s_gpio = gpio;
    s_active_low = active_low;
    s_on_press = on_press;
    s_ctx = ctx;
    /* A button held at startup does not trigger a press. */
    s_stable_pressed = s_last_raw = read_pressed();
    return ESP_OK;
}

void button_process(uint32_t now_ms)
{
    if (s_gpio == GPIO_NUM_NC) {
        return;
    }

    bool raw = read_pressed();
    if (raw != s_last_raw) {
        s_last_raw = raw;
        s_last_change_ms = now_ms;
        return;
    }
    if (raw == s_stable_pressed || now_ms - s_last_change_ms < DEBOUNCE_MS) {
        return;
    }

    s_stable_pressed = raw;
    if (s_stable_pressed && s_on_press) {
        s_on_press(s_ctx);
    }
}
