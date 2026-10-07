/*
 * Debounced push button, read by polling.
 *
 * The module is clocked by button_process(), called from the application's
 * event loop; the callback is therefore invoked in that context.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*button_press_cb_t)(void *ctx);

esp_err_t button_init(gpio_num_t gpio, bool active_low, button_press_cb_t on_press, void *ctx);

/* Call periodically with the current clock (ms). */
void button_process(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
