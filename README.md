# ESP AVB Endpoint

An AVB endpoint implementation using the esp_avb component for ESP-IDF.
Builds for two targets:

- **ESP32-P4** — wired AVB endpoint with on-chip IEEE 1588 hardware
  timestamping. Full talker/listener with codec.
- **ESP32-C6** — wireless AVB endpoint over Wi-Fi STA. Same talker/
  listener stack on the wireless data plane, paired with an
  ESP-AVB-Bridge that bridges AVB Wi-Fi onto a wired AVB
  switch.

The source lives at <https://github.com/scrambletools/ESP-AVB-Endpoint>;
old github.com/scrambletools/... links redirect there. The esp_avb,
esp_ptp and esp_hosted_avb components come from the ESP Component
Registry under the **scrambletools** namespace
(`scrambletools/esp_avb`, `scrambletools/esp_ptp`,
`scrambletools/esp_hosted_avb`), declared in `main/idf_component.yml`.

Hardware:

- **Wired endpoint (ESP32-P4):** Scramble offers developer hardware
  with this firmware pre-loaded at <www.scramble.tools>. You can also
  get a Waveshare ESP32-P4-ETH from other vendors and flash it
  yourself.
- **Wireless endpoint (ESP32-C6):** any ESP32-C6 dev board. Pair it
  with an ESP-AVB-Bridge (see scrambletools/ESP-AVB-Bridge) to reach
  the wired AVB network. Codec/I2S can be left disabled for boards
  with no audio hardware (the c6 default in `sdkconfig.defaults.esp32c6`
  builds without codec for that reason).

This AVB implementation is based on the following standards:

- IEEE 1722-2016 (AVTP)
- IEEE 1722.1-2021 (ATDECC)
- IEEE 802.1Q-2022 (MSRP, MVRP)
- IEEE 802.1AS-2021 (gPTP, based on ESP-IDF PTPd implementation)
- IEEE 802.11 (FTM peer-delay, beacon Vendor IE FollowUpInformation
  carriage)

Currently supports:

- AVB talker and listener (both targets)
- Simultaneous input and output stream
- Class A or B streams over Ethernet, Class B streams over Wi-Fi
- 24bit/48kHz-192kHz audio in AAF or AM824
- Control via ATDECC controller (tested with Hive)
- Wi-Fi STA endpoint with software-disciplined PTP clock, FTM
  peer-delay initiator, and beacon-IE FollowUpInformation consumer

Anticipated future support:

- ES8388/9 hat for ESP32-P4-ETH (to be designed)
- Milan 1.3 certification or at least compatibility
- AVB community audio profile support (in draft)
- AVB Lite (works with any switch, no bandwidth guarantee)

## Building

Pick the target before building:

```
# Wired endpoint (default):
idf.py set-target esp32p4
idf.py build flash monitor

# Wireless endpoint:
idf.py set-target esp32c6
idf.py build flash monitor
```

Per-port topology lives in `esp_ptp`'s Kconfig and is set per target
via `sdkconfig.defaults.esp32c6` (Wi-Fi STA endpoint). The matching
`esp_avb` symbols are derived automatically — there is no separate AVB
role switch.

## About this example

This application can operate as talker and/or listener. It uses the
esp_avb component which currently supports the Everest ES8311 mono
CODEC at 24bit/48kHz PCM audio (in either AAF or AM824/IEC61883-6
stream format) on targets with audio hardware. The example demonstrates
the use of the esp_avb component with very little knowledge of AVB
needed; our hope is that it shows how esp_avb can be dropped into an
existing ESP-IDF audio application to add AVB connectivity for
realtime low-latency audio routing.

The same `main/avb_endpoint.c` source serves both targets — the
medium-specific bring-up (Ethernet vs Wi-Fi STA + FTM + beacon-IE
consumer) is gated by `CONFIG_ESP_PTP_PORT0_MEDIUM_*`.

## Controller

There is a simple command-line ATDECC controller in the
avbcommunity/tools repo for basic connectivity testing. The main
application is intended to showcase the talker and listener
functionality of AVB, so it requires a controller to make a
connection. It has been tested with the Hive AVB controller
(<https://github.com/christophe-calmejane/hive>), the avb_tools
controller, and the Apple ATDECC controller (built into MacOS).

## Hardware notes

- **ESP32-P4 wired endpoint:** tested on the Waveshare ESP32-P4-ETH
  board, and should work with any board pairing the ESP32-P4 with the
  ES8311 codec. The ESP32-P4 is required for wired operation because
  it has the on-chip IEEE 1588 hardware timestamping; older ESP32 SoCs
  with EMAC do not support hardware timestamping and will not meet the
  AVB sync precision requirements.
- **ESP32-C6 wireless endpoint:** tested on a generic ESP32-C6 dev
  board. The c6 has no on-chip MAC, so the PTP daemon disciplines a
  software clock backed by `esp_timer` (`ptp_clock_sw.c`). Time sync
  arrives out-of-band: 802.11 beacon Vendor IE for FollowUpInformation
  and FTM for peer-delay. The wireless endpoint requires an AVB bridge
  to terminate Wi-Fi AVB onto a wired AVB switch — see
  scrambletools/ESP-AVB-Bridge.

Compatibility testing has been performed with the MOTU AVB Switch, the
MOTU 8D audio interface, and the Apple virtual AVB endpoint device
(part of MacOS).

## Open source

This application and the esp_avb component were initially developed by
Scramble Tools LLC, and they are provided as open source software for
the AVB/Milan developer community under the MIT software license to
encourage further development of low cost products and solutions for
AVB audio networking.

## Feedback

Please provide feedback or pull requests via the Github repository.
