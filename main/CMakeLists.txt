idf_component_register(SRCS "app_main.c"
                            "ble_hid_kbd.c"
                            "led.c"
                            "button.c"
                       PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_timer
                       INCLUDE_DIRS ".")
