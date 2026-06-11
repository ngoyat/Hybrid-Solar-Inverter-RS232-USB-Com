# Hybrid Solar Inverter RS232 (USB) Com

> **ESP8266 (Wemos D1 Mini) · Modbus RTU · RS232 via USB-A Socket · MQTT**

Read live data from a hybrid solar inverter over its RS232 Modbus port and publish it to an MQTT broker (e.g. Mosquitto on Raspberry Pi) for use with Node-RED, Home Assistant, or any MQTT-compatible dashboard.

---

## ⚠️ Disclaimer & Warning

> **USE ENTIRELY AT YOUR OWN RISK.**
>
> This project involves interfacing with electrical equipment including solar inverters, batteries, and mains-connected devices. Incorrect wiring, wrong voltage levels, or any modification to your inverter can cause **permanent equipment damage, electric shock, fire, or personal injury**.
>
> **The author of this repository takes absolutely no responsibility for any damage — to equipment, property, or persons — that may result from using, misusing, or modifying this code or wiring.** No warranty of any kind is provided. This project is shared purely as a personal experiment. You are solely responsible for your own actions.
>
> If you are not confident working with electronics, **do not proceed.**

---

## About the Inverter Communication Port

These hybrid solar inverters use a **USB Type-A socket** on the inverter body as their communication port — but it is **NOT a standard USB port**. It carries **RS232 signals** only and is completely incompatible with USB devices.

> ✅ **How to confirm:** Measure Pin 1 (VBUS) of the socket against GND with a multimeter while the inverter is powered on. A standard USB port reads **+5 V**. If it reads a **negative voltage** (typically −5 V to −10 V), it is definitively RS232. No USB device should ever be plugged into this port.

Because the connector is USB-A but the signal is RS232, the adapter chain is:

```
Inverter USB-A socket  →  USB-A to USB-A cable  →  MAX3232 RS232 side
```

The MAX3232 board converts the ±5–15 V RS232 levels down to 3.3 V TTL for the Wemos D1 Mini.

---

## Compatible Hardware

This code was developed and tested with an inverter built around a **high-frequency hybrid inverter PCB** — the type commonly found in 3–6 kW hybrid solar inverters sold under various brands across South/South-East Asia. The board features a toroidal transformer, PFC stage, full-bridge MOSFET topology, and exposes Modbus RTU communication via the USB-A RS232 port.

![Inverter PCB](images/inverter_board.png)

> *Typical hybrid solar inverter PCB that this code works with. The USB-A socket on the inverter chassis connects to this board's RS232 interface.*

If your inverter has this style of port and communicates via Modbus RTU at **9600 baud, 8N1**, this code is very likely compatible.

Tested With Model :
Gootu GT-H4865M27P9 6.5Kw Solar Hybrid Inverter Supporting 9000w PV
---

## Hardware Required

| Component | Details |
|---|---|
| Wemos D1 Mini | ESP8266-based, 3.3 V logic |
| MAX3232 mini board | RS232 ↔ TTL level shifter (3.3 V compatible) |
| USB-A to USB-A cable | Connects inverter RS232 port to MAX3232 RS232 side |
| Inverter with USB-A RS232 port | Modbus RTU, 9600 8N1 |
| Jumper wires | — |

---

## Wiring

![Wiring Diagram](images/wiring_diagram.png)

> *Wiring: USB-A cable from inverter → MAX3232 mini board RS232 side → Wemos D1 Mini TTL side. Use same-side pads on the MAX3232 board for RS232 and TTL connections.*

```
Inverter          USB-A Cable       MAX3232 Board            Wemos D1 Mini
────────          ───────────       ─────────────            ─────────────
USB-A TX(D−) ──── White Wire ──   RS232-RX  →  TTL       ──  D6 (GPIO12) 
USB-A RX(D+) ──── Green Wire ──   RS232-TX  ←  TTL       ──  D5 (GPIO14) 
USB-A GND    ──────────────────    GND             GND   ──  GND
                                   VCC             3V3   ──  3V3
USB-A VCC     ────────────────── ────────────────── ──   ──  5V
```
Take Ref from Picture , MAX3232 Board RX TX is marked as  ──► and can confuse

> **Important:** Connect RS232 side and TTL side to the **same side pads** of the MAX3232 mini board as labelled. Do not cross RS232 and TTL pads.

---

## Libraries

Install these via **Arduino IDE → Library Manager**:

- `PubSubClient` by Nick O'Leary
- `ESP8266WiFi` — bundled with the ESP8266 Arduino board package

---

## Configuration

Open `inverter_monitor/inverter_monitor.ino` and edit only this block at the top:

```cpp
// WiFi
#define WIFI_SSID        "YourWiFiSSID"
#define WIFI_PASSWORD    "YourWiFiPassword"

// MQTT Broker
#define MQTT_SERVER      "192.168.1.x"   // IP of your MQTT broker
#define MQTT_PORT        1883
#define MQTT_USER        ""              // leave empty if no auth
#define MQTT_PASS        ""              // leave empty if no auth
#define MQTT_CLIENT_ID   "inverter_d1"   // unique device name
#define MQTT_BASE_TOPIC  "inverter"      // root topic prefix

// Polling
#define POLL_SECONDS     10              // how often to read (seconds)
```

That's all you need to change. Do **not** modify register addresses unless you know what you're doing.

---

## MQTT Topics Published

All values are published under `<MQTT_BASE_TOPIC>/` with `retain = true`.

| Sub-topic | Unit | Description |
|---|---|---|
| `status` | `online` | Published on connect |
| `mains_voltage` | V | Grid/mains voltage |
| `mains_frequency` | Hz | Grid frequency |
| `grid_power_w` | W | Grid power import/export |
| `pv1_voltage` | V | Solar panel voltage |
| `pv1_current` | A | Solar panel current |
| `pv1_power_w` | W | Solar panel power |
| `daily_pv_kwh` | kWh | PV energy generated today |
| `total_pv_kwh` | kWh | Total lifetime PV energy |
| `battery_voltage` | V | Battery terminal voltage |
| `battery_soc` | % | Battery state of charge |
| `bus_voltage` | V | DC bus voltage |
| `battery_chg_amps` | A | Charge current (positive when charging, else 0) |
| `battery_dis_amps` | A | Discharge current (positive when discharging, else 0) |
| `battery_current` | A | Signed current (+ discharge / − charge) |
| `charging_status` | 0–3 | 0=None 1=CV/CC 2=Float 3=Equalize |
| `ac_load_w` | W | AC output load power |
| `daily_output_kwh` | kWh | AC energy output today |
| `inv_temp_c` | °C | Inverter heat-sink temperature |
| `chg_temp_c` | °C | Charger heat-sink temperature |
| `fan_speed_pct` | % | Cooling fan speed |

---

## Flash Wear Protection

The sketch calls `WiFi.persistent(false)` before every `WiFi.begin()`. This prevents the ESP8266 SDK from writing WiFi credentials to flash on every connect/reconnect, protecting the flash chip from unnecessary write cycles. No EEPROM, SPIFFS, or LittleFS is used anywhere in this sketch — all data lives in RAM.

---

## License

MIT License

Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy of this software to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. THE AUTHOR SHALL NOT BE HELD LIABLE FOR ANY DAMAGES ARISING FROM THE USE OF THIS SOFTWARE.
