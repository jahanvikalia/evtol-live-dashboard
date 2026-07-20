# eVTOL Live Telemetry & Landing Safety Dashboard

A real-time, browser-based flight telemetry dashboard for an eVTOL prototype, built on ESP32. Onboard sensors are read continuously and served over WiFi as a live web dashboard — attitude (pitch/roll), speed estimation, GPS position on a map, temperature, and a 5-point IR landing-safety array — with buzzer/LED alerting for hazard conditions.

**Tech stack:** ESP32 · Arduino (C/C++) · I2C · UART · OneWire · WiFi (WebServer) · Leaflet.js

 &nbsp;|&nbsp; 💼 **[demo video on LinkedIn](https://www.linkedin.com/posts/jahanvi-kalia_evtol-iot-embeddedsystems-activity-7366125087588089857-izSj)**

## Overview

Built during a Summer Internship at Thapar Institute of Engineering & Technology's ELC Division, under the eVTOL Research Project. The ESP32 acts as both a sensor hub and a web server: it polls all onboard sensors, computes derived values (attitude, velocity estimate, movement state), and serves a live dashboard to any browser on the same network — no app or PC software required.

## Sensors & Interfaces

| Sensor | Interface | Purpose |
|---|---|---|
| MPU6050 (IMU) | I2C (SDA 19 / SCL 18) | Acceleration + gyro → pitch, roll, movement state, speed estimate |
| DS18B20 | OneWire (GPIO 21) | Live temperature readout |
| 5× IR sensors | Digital GPIO (33, 32, 27, 26, 25) | Landing-surface safety detection |
| NEO-6M GPS | UART2 (GPIO 16/17) | Live position, plotted on an embedded Leaflet map |
| RGB LED + Buzzer | Digital GPIO | Visual/audible hazard alerting |

## How It Works

1. ESP32 connects to WiFi (falls back to a self-hosted Access Point if no network is found)
2. Sensors are polled continuously in the main loop; IMU data is converted into pitch/roll and a damped velocity estimate
3. A lightweight web server exposes:
   - `/` — the dashboard UI (single-page, no external hosting needed)
   - `/data` — a JSON endpoint with live sensor readings, polled by the frontend every 500ms
4. The dashboard renders live gauges, an animated attitude indicator, a GPS map (via Leaflet.js/OpenStreetMap), and IR-based landing-safety status
5. If any IR sensor flags a hazard, or acceleration exceeds a fast-movement threshold, the system triggers the buzzer + red LED and flags an alert on the dashboard

## Setup

1. Install Arduino libraries: `WiFi`, `WebServer`, `Wire`, `OneWire`, `DallasTemperature`, `TinyGPS++`
2. Copy `secrets.h.example` → `secrets.h` and fill in your WiFi credentials (this file is gitignored and won't be committed)
3. Wire sensors per the pin definitions at the top of the `.ino` file
4. Flash to an ESP32
5. Open the Serial Monitor to find the assigned IP address, then open that IP in a browser

## Files

- `evtol_dashboard.ino` — full firmware + embedded dashboard frontend
- `secrets.h.example` — WiFi credential template (copy to `secrets.h` locally)

## Demo

See the video linked above for a walkthrough of the live dashboard in action.
