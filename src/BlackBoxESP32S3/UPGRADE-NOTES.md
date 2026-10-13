# Black Box firmware v1.1.0 — pass 1 notes

## Before you build

1. **Partition scheme (required for /update to work).** In Arduino IDE → Tools → Partition Scheme, pick one with two app slots on 16 MB flash, e.g. **"16M Flash (3MB APP/9.9MB FATFS)"** (core 3.x) or **"16M Flash (3MB APP/9MB FATFS)"** (core 2.x). The current build uses "Huge APP (3MB No OTA)", which has a single `app0` and `Update.begin()` will fail with "No OTA partition". The first flash with the new scheme must be over USB; after that, updates go through the portal.
2. Replace your `lv_conf.h` with the one in this folder. Unused widgets/themes are turned off; `LV_HOR_RES_MAX`/`LV_VER_RES_MAX` (v8-only) are gone. If the build complains about a missing widget you add later, flip it back on.
3. Tune `MIN_PUMP_PCT` (default 10). Anything below it is floored, and presets below it are rejected / dropped on load.
4. Pick which `black_box.c` is your live logo — two different ones were in the upload (181 KB vs 46 KB).
5. Set `FIRMWARE_VERSION` and `manifest.json` to the same value when you publish.

## What changed

**Fixed**
- Arc/slider to 0 % now stops the pump (previously resumed at last speed on the next ramp tick).
- Web PLUG toggles are queued to the plug task; they no longer freeze the screen/ramp for up to ~9 s.
- On-screen PLUG buttons reflect toggles made from the web page.
- `noInterrupts()` critical sections removed (per-core only on ESP32); flags are `std::atomic`.

**Added**
- Task watchdog (10 s) on the main loop — a hang reboots with the pump off.
- `/update` firmware upload page on the portal (stops pump, flashes, reboots). Linked from the web footer and Help screen.
- Minimum pump speed floor.
- Plug API format (Shelly Gen1 / Gen2 / Tasmota) remembered per plug after first success.
- `/cmd` also accepts GET: `curl "http://192.168.3.1/cmd?S=50"` — handy for show-control integrations.
- `/state` now includes `target` and `fw`.

**Changed**
- `/cmd` takes form params (`S`, `M`, `P1`, `P2`, `AP`, `DP`) instead of hand-parsed JSON, and replies with fresh state.
- `/state` built with `snprintf` (no `String` heap churn under 700 ms polling).
- AP max clients 4 → 8. SSID suffix alphabet drops 0/O and 1/I.
- Pump GPIOs driven safe as the first statement in `setup()`.
- I2C 100 kHz → 400 kHz (touch latency).
- LVGL: `lv_tick_set_cb`, `LV_EVENT_CLICKED` filters instead of `LV_EVENT_ALL`, loop sleeps per `lv_timer_handler()`.
- Web slider: live local feedback, sends throttled to ~120 ms, polling paused while dragging, slider tracks target (not the ramping value).
- Heap print is behind `DEBUG_HEAP`.

## Pass 2 (bench-test separately)
- `USE_HW_SPI 1` — hardware SPI via the S3 GPIO matrix at 40 MHz instead of bit-banged SWSPI. Expect a large redraw speedup; verify the panel is happy.
- Once on HW SPI: `buf_h` 20 → 40 and pass a second buffer to `lv_display_set_draw_buffers`.
- `PUMP_PWM_FREQ_HZ` 2000 → 20000 to kill the audible whine (check low-speed starting torque on your pump).

## Not compiled here
I don't have the ESP32 toolchain + Arduino_GFX/LVGL/XPowersLib in this environment, so this was reviewed by hand, not compiled. Expect at most minor API-name nits on first build; send me the errors if any.
