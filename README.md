# SurgeFX Black Box — firmware

ESP32-S3 pump controller: LVGL touchscreen, WiFi access point with captive-portal web UI, BTS7960 (IBT-2) motor driver with soft ramp, smart-plug control, presets in NVS, and local firmware update over the portal.

- `src/BlackBoxESP32S3/` — Arduino sketch (open the `.ino` in Arduino IDE)
- `BlackBoxESP32S3.ino.bin` + `manifest.json` — latest release binary, served by GitHub Pages for customer downloads

## Hardware

- Waveshare **ESP32-S3-Touch-LCD-3.5** (ST7796 display, FT6336 touch, TCA9554 expander, AXP2101 PMIC)
- **IBT-2** (BTS7960) H-bridge, forward direction only
- Shelly / Tasmota smart plugs at 192.168.3.101 and .102 on the Black Box network

### Wiring: Waveshare rear header → IBT-2

| Signal | Waveshare pin | IBT-2 pin |
|---|---|---|
| RPWM (speed) | 10 | 1 RPWM |
| LPWM (held low) | 11 | 2 LPWM |
| R_EN | 41 | 3 R_EN |
| L_EN | 42 | 4 L_EN |
| R_IS (optional, future current sense) | 9 | 5 R_IS |
| VCC | 3V3 (5V also works) | 7 VCC |
| GND | G | 8 GND |

IBT-2 pin 6 (L_IS) unconnected. Pump supply to **B+/B−**, pump to **M+/M−** — swap M+/M− if it runs backwards. Pins 10/11 are shared with the SD slot; don't insert a card.

## Building (Arduino IDE 2.x)

ESP32 core **3.3.2** (3.3.12 breaks the installed Arduino_GFX; either pin the core or update GFX). Libraries: lvgl 9, GFX Library for Arduino, XPowersLib, TCA9554. Copy `lv_conf.h` from the sketch folder into your Arduino `libraries/` folder (next to the `lvgl` folder).

Tools menu:

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| Flash Mode | QIO 80MHz |
| Flash Size | 16MB (128Mb) |
| PSRAM | OPI PSRAM |
| Partition Scheme | 16M Flash (3MB APP/9.9MB FATFS) |
| Upload Mode | UART0 / Hardware CDC |
| Erase All Flash Before Sketch Upload | Disabled (Enabled once when changing partition scheme) |

Export the binary with **Sketch → Export Compiled Binary**; it lands in `src/BlackBoxESP32S3/build/`. Copy the `.ino.bin` to the repo root and bump `manifest.json` to release.

## Updating a device in the field

The Black Box is AP-only, so updates are a two-step for the customer:

1. On cellular or home WiFi, download `BlackBoxESP32S3.ino.bin` from the link on surgefx.com.
2. Join the `SurgeFX-BlackBox-XXXX` network (password `blackbox123`), open `http://192.168.3.1/update`, choose the file, upload. The pump stops, the device flashes and reboots.

Current version is shown on the Help screen and at the bottom of the web page.

## Web API

`POST` or `GET /cmd` with form/query params; replies with current state JSON.

| Param | Meaning |
|---|---|
| `S=<0-100>` | set speed (0 = stop) |
| `M=<0|1>` | pump off / on at last speed |
| `P1=<0|1>`, `P2=<0|1>` | plug 1 / 2 |
| `AP=<pct>` | add preset |
| `DP=<index>` | delete preset |

`GET /state` → `{"speed","target","pump","p1","p2","presets":[],"fw"}`

## Tunables (top of the sketch)

`MIN_PUMP_PCT` (speed floor), `PUMP_PWM_FREQ_HZ`, `RAMP_UP_PCT_PER_S` / `RAMP_DOWN_PCT_PER_S`, `AP_MAX_CLIENTS`, `USE_HW_SPI` (hardware SPI for the display; bench-test before enabling).
