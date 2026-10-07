# esp32-fakeBLEkeyboard

Fake Bluetooth LE keyboard, based on ESP32-C3 SuperMini. 
Once paired with a Windows PC, it appears as a HID keyboard device. 
Randomly presses F15 key (every 15-30s).

## Behavior

| Situation                          | LED                         |
|------------------------------------|-----------------------------|
| No host paired                     | off                         |
| Pairing in progress (BOOT pressed) | blinks 4 times per second   |
| Paired, waiting for connection     | off                         |
| Connected                          | steady                      |
| Sending F15                        | brief blackout              |

- Pressing **BOOT** erases the existing pairing and opens a pairing window.
  It closes as soon as a host is paired.
- Pairing keys are persisted in NVS: on reboot, the PC reconnects on its own.
- Outside the pairing window, any new host attempting to pair is rejected.

## Architecture (`main/`)

| File            | Role |
|-----------------|------|
| `app_main.c`    | Application: state machine, **single event loop**, F15 timer |
| `ble_hid_kbd.*` | NimBLE stack, GATT services (HID, Device Info, Battery), security/bonding, key sending |
| `led.*`         | Non-blocking LED (off / steady / blinking / pulse) |
| `button.*`      | Debounced button, polled by the loop |

The modules do not know about each other. They report their events via callback;
the application posts them into a FreeRTOS queue and handles them in its loop, which also
clocks the LED, the button and the timer. The only other task is the NimBLE host task, imposed by the stack.

## Build and flash

```sh
. ~/.espressif/tools/activate_idf_v6.1.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

The BLE configuration is in `sdkconfig.defaults`.

## Pairing with Windows

1. Press BOOT: the LED blinks.
2. Windows: Settings → Bluetooth & devices → Add device → Bluetooth → "BLE Keyboard".
3. The LED turns steady.

If you restart pairing on the board, first remove the old "BLE Keyboard" in Windows:
Windows keeps its keys, which no longer match those on the board.
