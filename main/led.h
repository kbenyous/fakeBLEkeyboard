/*
 * Non-blocking LED control (steady, off, blinking, brief pulse).
 *
 * The module has neither a task nor a timer: it is clocked by led_process(),
 * called from the application's event loop.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_init(gpio_num_t gpio, bool active_low);

void led_off(void);
void led_on(void);

/* Continuous blinking: full period in ms (50% duty cycle). */
void led_blink(uint32_t period_ms);

/* Single pulse: inverts the current state for duration_ms, then returns to the current mode. */
void led_pulse(uint32_t duration_ms);

/* Call periodically with the current clock (ms). */
void led_process(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
