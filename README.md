# WaterMeter_flowiq2200_wmbus
WaterMeter-FlowIQ2200 (ESP32 + CC1101 + Home Assistant)
# FlowIQ 2200 for ESPHome

An ESPHome port of [erikxson/watermeter-flowiq2200](https://github.com/erikxson/watermeter-flowiq2200): read a **Kamstrup FlowIQ 2200** water meter over wM-Bus (mode C, 868.95 MHz) with an **ESP32 + CC1101**, and get the values straight into Home Assistant via the ESPHome integration.

Same hardware and AES decryption as the original. Telegrams are decoded with the field definitions of the wmbusmeters `kamwater` driver, so you get every value the meter sends (flow, daily min/max, temperatures, alarms). Wi-Fi, OTA, Home Assistant discovery, availability and the reset command are handled by ESPHome.

## Files

```
wasserzaehler.yaml            device config (edit names/pins here)
components/flowiq2200/        the external component
  __init__.py                 YAML schema + code generation
  flowiq2200.h/.cpp           CC1101 driver + telegram handling
  flowiq_decoder.h/.cpp       AES-128-CTR, CRC, DIF/VIF + compact frame decoder
```

## Setup

1. Copy `watermeter.yaml` and the `components/` folder into your ESPHome config folder (in Home Assistant: `/config/esphome/`).
2. fill in:
   - `flowiq_meter_id`: the 8-digit meter number, **quoted**. It's the same bytes you had in `credentials.h`: `{ 0x53, 0x48, 0x08, 0x78 }` → `"53480878"`.
   - `flowiq_aes_key`: the 32-hex-character AES key from your utility, **quoted**.
3. Install from the ESPHome dashboard (first flash over USB, then OTA).
4. Home Assistant discovers the device automatically.

## Wiring

| CC1101 | ESP32 |
|---|---|
| VCC | 3V3 (**never 5V**) |
| GND | GND |
| MOSI | GPIO23 |
| SCK | GPIO18 |
| MISO | GPIO19 |
| GDO0 | GPIO26 |
| CSN | GPIO27 |
| GDO2 | not connected |

## Entities

Every entity is optional: delete the ones you don't want from the YAML.

**Readings**

| YAML key | Unit | wmbusmeters field |
|---|---|---|
| `volume` | m³ | `total_m3` (`total_increasing`, usable in the Energy dashboard) |
| `month_start` | m³ | `target_m3`, the reading on the target (billing) date |
| `target_date` | text | `target_date`, e.g. `2022-03-01` |
| `flow` | L/h | `flow_m3h` (current flow) |
| `max_flow_last_day` / `min_flow_last_day` | L/h | `max_flow_last_day_m3h` / `min_flow_last_day_m3h` |
| `min_water_temperature_last_day` / `max_water_temperature_last_day` | °C | `min/max_flow_temperature_last_day_c` |
| `min_ambient_temperature_last_day` / `max_ambient_temperature_last_day` | °C | `min/max_external_temperature_last_day_c` (your meter sends only the min) |
| `rssi` | dBm | radio signal of the last valid telegram |

**Status and alarms**

| YAML key | Type | Meaning |
|---|---|---|
| `status` | text | `OK`, or the active flags, e.g. `LEAK LOW_BATTERY` |
| `leak`, `burst`, `dry`, `reverse`, `tamper`, `low_battery`, `ambient_temperature_alarm`, `flow_above_q4`, `no_consumption` | binary | one per status flag |
| `time_leaking`, `time_bursting`, `time_dry`, `time_reversed`, `time_ambient_temperature`, `time_flow_above_q4` | text | how long the condition has lasted: `none`, `1-8 hours`, `9-24 hours`, `2-3 days`, `4-7 days`, `8-14 days`, `15-21 days`, `22-31 days` |
| `acoustic_noise` | text | raw 6-byte value; its meaning isn't publicly documented |

Plus the ESPHome `Online` status sensor and `Restart` button, which replace the original's `watermeter/0/online` and `watermeter/0/cmd/reset` topics. Prefer MQTT over the native API? Replace `api:` with an `mqtt:` block; ESPHome does the HA discovery for you.

## How decoding works

The meter sends two kinds of telegram:
- **Full frames** (CI 0x78) contain normal DIF/VIF records and are decoded directly.
- **Compact frames** (CI 0x79), the frequent ones, contain only the values plus a 2-byte *format signature*. The signature is the CRC of the matching full frame's DIF/VIF headers. The component knows all layouts from the wmbusmeters `kamwater` driver (including your meter's, signature `F3A9`), and learns any other layout from the first full frame it receives.

## What changed compared with the original

**Flow is read as its real 16-bit value.** In your meter's compact frame the flow is two bytes (L/h) right after the target date. The original read only the low byte; the "DIF 0x41 VIF 0x31" pattern its comments describe was actually the date bytes. So it saw 0–255 L/h and needed wrap correction, full-frame offsets and caching to guess anything higher. All of that is gone: every frame now carries the exact flow. The old `flow_full` option is no longer needed and ESPHome will point that out if it's still in your YAML.

**Full frames are no longer truncated.** The original waited for the CC1101's 64-byte FIFO to overflow before reading, cutting off anything longer. This version drains the FIFO while the telegram arrives.

**Other differences**
- The CC1101 is checked at boot; with bad wiring the log says so instead of hanging.
- A watchdog restarts the receiver if the radio ever leaves RX mode.
- Debug output uses ESPHome log levels: `DEBUG` shows each decoded frame, `VERBOSE` also lists telegrams from other meters (handy for finding your meter ID), `VERY_VERBOSE` dumps decrypted payloads.
- ESP32 only (the original's ESP8266 target was untested anyway).

## Troubleshooting

- **`CC1101 not responding`** — check wiring and that the module is on 3.3V.
- **`Payload CRC mismatch ... wrong AES key`** — the meter ID matched, but decryption failed: check the key.
- **Nothing at all** — set `logger: level: VERBOSE`. If you see "Ignoring telegram from meter …" lines, the radio works and your `meter_id` doesn't match any of them. If you see nothing, check the antenna (868 MHz; a straight wire of ~8.6 cm works) and the distance to the meter.
- **`unknown format signature`** — your meter uses a layout not in the built-in list. It is learned automatically from the next full frame; until then compact frames are skipped.
- **`RX FIFO overflow`** — the main loop was too slow to drain the radio. Occasional ones are harmless; frequent ones mean something else in the config is blocking the loop for long periods.

## License

GPL-3.0-or-later, like the projects it is derived from: watermeter-flowiq2200 by erikxson, esp32-multical21 by pthalin, and esp-multical21 by chester4444. Field definitions follow the wmbusmeters `kamwater` driver by Fredrik Öhrström (GPL-3.0-or-later). See `LICENSE`.
