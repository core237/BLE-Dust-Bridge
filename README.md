# BLE-Dust-Bridge

A BLE bridge for controlling Festool dust extractors on an M5Stack AtomS3 Lite (ESP32-S3) using Festool CT-F remotes, passive/GATT wireless PTT switches, Festool Bluetooth batteries, and Shelly Gen3/Gen4 power-monitoring smart plugs.

![Supported Hardware Lineup](assets/hardware-lineup.jpg)

## Overview & Features

**BLE-Dust-Bridge** turns a compact M5Stack AtomS3 Lite into an automated dust-collection hub for your workshop—bridging wireless triggers and corded power tools (such as a Shaper Origin) to your Bluetooth-enabled Festool extractor without hardware modifications:

* **Multi-Trigger Automation:** Automatically starts and stops your dust extractor from **CT-F** remotes, passive burst or **GATT-only PTT** wireless buttons, **Festool Bluetooth batteries**, and **Shelly Gen3/Gen4** power-monitoring plugs.
* **Smart Wattage & Ramp-Rate Filtering:** Monitors live tool power draw over BLE beacons (`shelly_dust_beacon.js`) with configurable wattage cutoffs (e.g., `45W–50W` to ignore idle draw and Z-axis servo spikes on a Shaper Origin) and configurable post-cut evacuation delay.
* **Built-In Web Dashboard & OTA Updates:** Configure devices, monitor live telemetry, and install firmware updates directly over Wi-Fi via the bridge's hosted web UI (`http://dustbridge.local`).
* **Home Assistant & MQTT Ready:** Publishes device states, extractor telemetry, and control topics over MQTT.

## Web Installer & OTA Firmware

* **Web Installer (Flash a fresh AtomS3 Lite via USB in Chrome/Edge):** https://core237.github.io/BLE-Dust-Bridge/
* **Latest OTA Binary (`firmware.bin` for Settings → Update):** https://github.com/core237/BLE-Dust-Bridge/releases/latest/download/firmware.bin

## Shelly Plug Script

* **Gen3 / Gen4 Power Beacon:** [`shelly/shelly_dust_beacon.js`](shelly/shelly_dust_beacon.js)
