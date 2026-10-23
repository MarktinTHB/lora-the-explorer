# Lora the Explorer

**A Decentralized Offline LoRa-Based Wearable Communication and Tracking System for Dead Zone Environments**

Lora the Explorer is a wearable, infrastructure-free tracking system. Three ESP32-based devices talk to each other directly over LoRa radio. They do not need cellular, Wi-Fi, internet, or a gateway. Each unit shares its GPS position, compass heading, and status with its peers. It then shows where the others are on a radar-style touchscreen display, with distance and relative direction.

The system was built as a proof-of-concept prototype for hikers, explorers, families in disaster-prone areas, and rescue teams. These are people who often find themselves in "dead zones" where conventional networks are unavailable.

> Undergraduate thesis project, Bachelor of Science in Computer Engineering, FEU Institute of Technology (2026).

---

## Table of Contents

- [Features](#features)
- [How It Works](#how-it-works)
- [Hardware](#hardware)
- [Pin Configuration](#pin-configuration)
- [Software and Libraries](#software-and-libraries)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Using the Device](#using-the-device)
- [Communication Protocol](#communication-protocol)
- [Timing and Timeouts](#timing-and-timeouts)
- [Calibration](#calibration)
- [Packet Test and Diagnostics](#packet-test-and-diagnostics)
- [Firmware Versioning](#firmware-versioning)
- [Testing Methodology](#testing-methodology)
- [Scope and Limitations](#scope-and-limitations)
- [Thesis vs. Current Firmware](#thesis-vs-current-firmware)
- [Authors](#authors)
- [References](#references)

---

## Features

- **Offline peer-to-peer communication.** Devices exchange data directly over LoRa at 915 MHz. No network infrastructure is involved.
- **GPS location sharing.** Each device reads its position from a NEO-6M GPS module and broadcasts it to its peers.
- **Compass heading.** A QMC5883L or HMC5883L magnetometer gives the device's heading. The firmware detects which chip is present automatically.
- **Distance estimation.** The firmware calculates the distance between devices from GPS coordinates using TinyGPS++'s great-circle (Haversine-style) distance function.
- **Radar-style directional guidance.** Peers appear as colored dots on a compass ring, positioned by their bearing relative to the wearer's current heading.
- **Device status monitoring.** Each peer is shown as **ONLINE**, **ASLEEP**, or **OFFLINE**, based on heartbeats and timeouts.
- **Ping.** Users can ping one peer or all peers. Targeted pings report a measured round-trip time.
- **Sleep mode.** The device stops regular heartbeats, puts the radio to sleep, and sends a compact location report every 5 minutes.
- **GPS time and date.** The device shows local time (UTC+8). If it has no GPS fix, it borrows the clock from a peer's heartbeat.
- **Touchscreen UI.** The UI runs on a 240×320 ILI9341 TFT with an XPT2046 resistive touch panel.
- **On-device calibration.** Touch and compass calibration are both stored in flash for each board.
- **Built-in packet test.** The test measures packet success rate, round-trip time, RSSI, and SNR, and writes CSV output over serial.

---

## How It Works

```
 ┌──────────────┐         LoRa 915 MHz          ┌──────────────┐
 │   LORA #1    │ ◄───────────────────────────► │   LORA #2    │
 │  (Device 1)  │                               │  (Device 2)  │
 └──────┬───────┘                               └──────┬───────┘
        │                                              │
        │                ┌──────────────┐              │
        └──────────────► │   LORA #3    │ ◄────────────┘
                         │  (Device 3)  │
                         └──────────────┘
```

Every device runs the same firmware. The only difference is its `DEVICE_ID`. The main loop on each device does the following:

1. **Reads the GPS** and accepts a fix only if it is valid, recent (≤ 5 s old), and not `0,0`.
2. **Reads the magnetometer** every 100 ms and computes a calibrated heading.
3. **Listens for LoRa packets** and handles heartbeats, sleep reports, status sync, pings, and test packets.
4. **Sends a heartbeat** every 3 seconds when awake. The heartbeat carries its ID, GPS fix, coordinates, time and date, and heading.
5. **Updates peer status.** A peer goes OFFLINE after missing heartbeats past the timeout.
6. **Handles touch input** and redraws only the parts of the screen that changed.

On the **Check Location** screen, the device calculates the distance and bearing to each peer from the two GPS positions. It subtracts its own compass heading and plots the peer on the radar ring.

### Block Diagram (from the thesis)

| Module | Component | Role |
|---|---|---|
| Central Processing and Control | ESP32 | Runs the firmware and coordinates all modules |
| Location Acquisition | NEO-6M GPS | Provides latitude, longitude, time, and date |
| Orientation and Direction Sensing | QMC5883L / HMC5883L | Provides heading for directional guidance |
| Long-Range Communication | RFM95W LoRa transceiver | Sends and receives peer data |
| User Interface and Display | ILI9341 TFT + XPT2046 touch | Shows the radar, status, and controls |
| Power Management | Rechargeable Li-ion battery + regulator | Powers the portable unit |

---

## Hardware

Each of the three units uses:

| Component | Part | Notes |
|---|---|---|
| Microcontroller | ESP32 dev board | Arduino-compatible |
| LoRa transceiver | RFM95W | 915 MHz, SF7, BW 125 kHz, CR 4/5, TX power 17 dBm |
| GPS | u-blox NEO-6M | 9600 baud on UART2 |
| Magnetometer | QMC5883L (0x0D) or HMC5883L (0x1E) | Detected automatically over I²C |
| Display | 2.8" ILI9341 TFT, 240×320 | Driven by a custom SPI driver in the firmware |
| Touch | XPT2046 resistive touch controller | Shares the SPI bus with the TFT and LoRa |
| Antenna | 915 MHz LoRa antenna | |
| Power | Li-ion battery + voltage regulator | |
| Enclosure | 3D-printed | Not waterproof |

> **Display note:** The thesis's methodology refers to an OLED display. The working prototype uses an ILI9341 color TFT with touch input instead.

---

## Pin Configuration

All three devices use the same wiring.

### Shared SPI bus (TFT, Touch, LoRa)

| Signal | ESP32 GPIO |
|---|---|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |

### ILI9341 TFT

| Signal | ESP32 GPIO |
|---|---|
| CS | 5 |
| DC | 2 |
| RST | 4 |

### XPT2046 Touch

| Signal | ESP32 GPIO |
|---|---|
| CS | 21 |

### RFM95W LoRa

| Signal | ESP32 GPIO |
|---|---|
| NSS / CS | 27 |
| RESET | 32 |
| DIO0 | 26 |

### NEO-6M GPS (UART2)

| GPS Pin | ESP32 |
|---|---|
| VCC | 5V |
| GND | GND |
| TX | GPIO 16 (ESP32 RX) |
| RX | GPIO 17 (ESP32 TX) |

### Magnetometer (I²C)

| Signal | ESP32 GPIO |
|---|---|
| SDA | 13 |
| SCL | 22 |

> SDA was moved off GPIO 12 on purpose. GPIO 12 is an ESP32 strapping pin, and using it can stop the board from booting from flash.

---

## Software and Libraries

**Development environment:** Arduino IDE with the ESP32 board package.

| Header | Library |
|---|---|
| `SPI.h`, `Wire.h`, `Preferences.h` | Built into the ESP32 Arduino core |
| `Adafruit_GFX.h` | Adafruit GFX Library |
| `XPT2046_Touchscreen.h` | XPT2046_Touchscreen (Paul Stoffregen) |
| `LoRa.h` | LoRa (Sandeep Mistry) |
| `TinyGPS++.h` | TinyGPSPlus (Mikal Hart) |
| `math.h` | Standard C library |

The ILI9341 is driven by a small custom class, `WorkingILI9341`, that extends `Adafruit_GFX`. A separate ILI9341 driver library is not needed.

---

## Repository Structure

```
.
├── LORA_THE_EXPLORER_DEV1.cpp   # Firmware for Lora #1 (DEVICE_ID 1)
├── LORA_THE_EXPLORER_DEV2.cpp   # Firmware for Lora #2 (DEVICE_ID 2)
├── LORA_THE_EXPLORER_DEV3.cpp   # Firmware for Lora #3 (DEVICE_ID 3)
└── README.md
```

The three firmware files share the same logic and protocol. Each one sets its own ID:

```cpp
#define DEVICE_ID 1   // 2 for DEV2, 3 for DEV3
```

> The packet-test output mentions a PC-side logger script (`tools/packet_logger.py`). That script is not included in this repository yet.

---

## Getting Started

1. **Install the Arduino IDE** and the **ESP32 board package** (Boards Manager → "esp32" by Espressif).
2. **Install the libraries** listed above through the Library Manager.
3. **Set up a sketch.** The Arduino IDE expects a `.ino` file inside a folder with the same name. For each device, for example:
   ```
   LORA_THE_EXPLORER_DEV1/
   └── LORA_THE_EXPLORER_DEV1.ino   ← contents of LORA_THE_EXPLORER_DEV1.cpp
   ```
4. **Select your ESP32 board** and its serial port.
5. **Upload** the right firmware to each unit. Flash `DEV1` to Lora #1, `DEV2` to Lora #2, and `DEV3` to Lora #3.
6. **Open the Serial Monitor** at **115200 baud** to see boot messages, received packets, and diagnostics.

> Never flash the same `DEVICE_ID` onto two units. Each device ignores packets that carry its own ID.

---

## Using the Device

### Main Menu

| Area | Action |
|---|---|
| **Time / Date** | Shows local time (UTC+8) from GPS, or a peer's clock if this device has no fix. **Double-tap** to run the packet test. |
| **CHECK LOCATION** | Opens the radar screen. |
| **SLEEP MODE** | Turns sleep mode on or off. |
| **Own device icon** | Pings **all** peers (broadcast). |
| **Peer device icon** | Pings **only** that peer (targeted). |
| **Status row** | Shows ONLINE (green), ASLEEP (yellow), or OFFLINE (gray) for LORA #1–#3. |

Device colors: **Lora #1 = red**, **Lora #2 = green**, **Lora #3 = blue**.

### Check Location (Radar) Screen

- The compass ring is fixed with N, E, S, and W labels. This device sits at the center.
- Each peer is drawn as a colored dot. Its angle is the peer's bearing relative to the current heading. Its radius is distance on a **fixed 100 m scale**, so peers farther than 100 m sit at the edge of the ring.
- Text rows show:
  - This device's coordinates, or `NO GPS FIX FOUND!`
  - This device's heading and direction, or `NO MAGNETOMETER FOUND!`
  - For each peer: `LORA #n | <distance> meters away! | <direction>`
- Distance shows `N/A` and direction shows `--` when the local or remote position is not valid or fresh, or when there is no magnetometer.
- Tap the **back arrow** (top left) to return to the main menu.
- **Hold the compass ring for 3 seconds** to start compass calibration.

---

## Communication Protocol

All packets are comma-separated ASCII strings. Device IDs are 1 to 3. Each device ignores packets with its own ID as sender.

| Type | Format | Purpose |
|---|---|---|
| Heartbeat | `HB,id,gpsFix,lat,lon,h,m,s,day,month,year,magPresent,heading` | Periodic presence, location, clock, and heading. Time fields are `-1` when unavailable. Heading is `-1` with no magnetometer. |
| Sleep report | `SLP,id,1,gpsFix,lat,lon,magPresent,heading` | Sent on entering sleep mode and every 5 minutes while asleep. |
| Status sync request | `SYNC,requesterId` | Sent when a device wakes up, asking awake peers for their state. |
| Status response | `STATE,sender,target,asleep,gpsFix,lat,lon,magPresent,heading` | Reply to `SYNC`. Only awake devices answer. |
| Ping | `P,requester,target,sequence` | `target = 0` means broadcast. Sequence is a non-zero 16-bit number. |
| Ping ACK | `PA,requester,target,sequence,responder` | Reply to a ping. Matched against the pending ping's requester, target, and sequence. |
| Test packet | `T,sender,seq` | Packet-test transmission. |
| Test reply | `TR,responder,to,seq,rssi,snr` | Echo carrying the RSSI and SNR the peer measured. |

### Ping Behavior

- **Targeted ping.** The sender records the send time and waits up to **3 s** for a matching `PA`. The round-trip time is measured with the sender's own `millis()`. The receiver waits about 60 ms before replying.
- **Broadcast ping.** The sender collects ACKs for **1.2 s**. Each responder waits `DEVICE_ID × 250 ms` before replying, so replies don't collide.
- **Duplicates.** Each receiver remembers the last 4 sequence numbers per sender and ignores repeats.
- Only one ping can be pending at a time. A new request shows `PING IN PROGRESS`.

### Location Validity

- A position is used only when the GPS fix is valid, the coordinates are finite and in range, and the position is not `0,0`.
- A remote location counts as **fresh** for **15 s** when the peer is awake, or **6 min** when it is asleep.
- A packet reporting "no fix" marks the peer's GPS as invalid. It does **not** overwrite the last known coordinates.
- GPS validity and heading validity are tracked separately for each peer.

---

## Timing and Timeouts

| Parameter | Value |
|---|---|
| Heartbeat interval | 3 s |
| Awake device timeout | 10 s (about 3 missed heartbeats) |
| Sleep location report interval | 5 min |
| Sleeping device timeout | 12 min (allows 2 missed reports) |
| Wake-up resync grace period | 8 s |
| Targeted ping ACK timeout | 3 s |
| Broadcast ping window | 1.2 s |
| Compass read interval | 100 ms |
| Forced screen refresh / display health check | 10 s |

### Sleep Mode

When a user turns on **Sleep Mode**:

1. The device saves the last known status of its peers. It shows that **frozen** status instead of marking peers offline while its own radio sleeps.
2. It sends an `SLP` report right away, puts the LoRa radio to sleep, and stops heartbeats.
3. Every 5 minutes it wakes the radio, sends another `SLP` report, and goes back to sleep.
4. Peers show the device as **ASLEEP** and keep its last location fresh for up to 6 minutes.

When sleep mode is turned off, the device wakes the radio, sends a heartbeat and a `SYNC` request, and waits through an 8-second grace period. This keeps peer statuses from flickering while fresh replies arrive.

---

## Calibration

Calibration data is stored in the ESP32's flash (`Preferences`) and belongs to that board only.

### Touchscreen

1. **Hold a finger on the screen while powering on.**
2. Release the screen when prompted, then tap the center of each of the 5 crosses.
3. The device fits an affine transform and saves it to the `touch` namespace.

If no saved calibration exists, the built-in default coefficients are used.

### Compass

1. Open **CHECK LOCATION** and **hold the compass ring for 3 seconds**.
2. Hold the device **flat** and rotate it slowly through 2–3 full circles for 20 seconds.
3. The device saves the hard-iron offsets and soft-iron scale factors to the `mag` namespace.

Calibration fails, and the old values are kept, if the X or Y range is under 200 raw counts. The device keeps sending heartbeats during calibration so its peers don't mark it offline.

---

## Packet Test and Diagnostics

**Double-tap the time** on the main menu to run the packet test. Sleep mode must be off.

- The device sends **20 test packets**, 300 ms apart.
- Each awake peer echoes every packet in its own time slot, so replies don't collide.
- The results screen shows, for each peer:
  - Packets received out of 20, and the success percentage
  - Average round-trip time
  - RSSI and SNR measured here and measured by the peer
  - GPS distance between the devices
- If another device is already running a test, this device waits instead of transmitting over it.

### Serial Output

The packet test prints machine-readable lines to the Serial Monitor (115200 baud):

```
CSV,device,test,seq,peer,status,rtt_ms,rssi_here,snr_here,rssi_there,snr_there,distance_m,own_lat,own_lon,peer_lat,peer_lon
SUM,device,test,peer,received,sent,success_pct,avg_rtt_ms,avg_rssi_here,avg_snr_here,avg_rssi_there,avg_snr_there,distance_m
```

Other diagnostic lines include:

- `LoRa RX: ...`: every received packet
- `PING RTT target=... seq=... rtt_ms=...`: completed targeted pings
- `GPS DIST ...`: distance and bearing to each peer, logged every 5 s
- `[status] TFT=0x.. #1=... #2=... #3=...`: device statuses every 10 s
- `TOUCH raw X=... -> screen X=...`: touch input mapping

---

## Firmware Versioning

Each firmware file should begin with a version header:

```cpp
// Current Version: vA.B.C.D
// Short summary of the feature or fix.
// Description of the important changes in this build.
```

| Field | Meaning |
|---|---|
| **A** | Development generation: `1` = earlier generation, `2` = current generation |
| **B** | Device ID: `1`, `2`, or `3` |
| **C** | Feature increment. Bumped for a new feature; D resets to 0. |
| **D** | Bug-fix counter within the current feature increment |

For example, `v2.1.1.0` is the first feature build for Device 1, and `v2.1.1.1` is its first bug fix. A change shared by all devices updates each file under its own device number (`v2.1.x.x`, `v2.2.x.x`, `v2.3.x.x`).

The version header doubles as the commit message for that build, so it should describe what actually changed.

> **Current baseline:** The firmware files in this repository do not have a version header yet. The first build under this convention will set the starting version.

---

## Testing Methodology

The thesis defines seven evaluation tests. All tests use the three prototypes in a dead-zone environment.

| # | Test | Success Criterion |
|---|---|---|
| 1 | Peer-to-peer exchange of GPS coordinates and device IDs | All three devices detect and display each other's IDs within 30 s in ≥ 9 of 10 trials |
| 2 | Distance estimation accuracy (100 m, 250 m, 500 m, 1000 m) | ≥ 8 of 10 trials per distance with error ≤ 20% (up to 500 m) or ≤ 30% (1000 m) |
| 3 | Directional guidance accuracy (8 directions at 50 m and 100 m) | Bearing error ≤ 15° in ≥ 8 of 10 trials |
| 4 | Packet success rate and effective range | PSR ≥ 90% in ≥ 9 of 10 trials at 100 m, 250 m, and 500 m. 1000 m is documented but not required. |
| 5 | Latency of location and ID transmission | Average ≤ 1000 ms over ≥ 27 of 30 trials, with no trial over 3000 ms |
| 6 | Sleep mode functionality (automatic and manual) | Sleep mode draws less power than active mode in ≥ 9 of 10 trials for each |
| 7 | Overall system performance | Average success rate of all core tests ≥ 90% |

**Formulas:**

- Percentage Error = |Displayed Distance − Actual Distance| / Actual Distance × 100
- Bearing Error = |Displayed Bearing − Actual Compass Bearing|
- Packet Success Rate = Packets Received / Packets Sent × 100
- Latency = Time of Reception − Time of Transmission

> Results will be added after field testing is complete. The results tables in the current thesis draft are not yet filled in.

---

## Scope and Limitations

- **Proof of concept.** This prototype is for research and demonstration only. It is not for real emergency use.
- **Three devices.** Testing used three units. Large networks, congestion, and scalability were not evaluated.
- **Location data only.** The system sends location and status information. It does not support voice, video, multimedia messaging, or internet services.
- **No route navigation.** The radar shows direction and approximate position. It does not provide routes or turn-by-turn navigation.
- **Environment-dependent.** GPS accuracy and LoRa range drop under dense vegetation, in mountainous terrain, near buildings, in bad weather, and with radio interference.
- **Not waterproof** without extra sealing. The rated operating range is 0 °C to 40 °C.
- **Unlicensed band.** The radio operates in the 915 MHz unlicensed band. It transmits only coordinates, heading, status, and device IDs. No data goes to cloud services.

---

## Thesis vs. Current Firmware

A few behaviors described in the thesis are not in the current firmware yet, or work differently:

| Thesis Description | Current Firmware |
|---|---|
| OLED display | ILI9341 color TFT with touch |
| **Automatic sleep** when all peers are within 50 m | Not implemented. Sleep mode is manual only. |
| Manual sleep wakes periodically to update GPS, exchange data, and refresh | The radio wakes every 5 min to send an `SLP` report and goes straight back to sleep. GPS and display keep running. |
| Freshness colors: green (< 1 min), yellow (1–10 min), red (> 10 min) | Peers are shown as ONLINE/ASLEEP/OFFLINE. Stale positions are hidden from the radar (15 s awake, 6 min asleep). |
| Battery status on the display | Not implemented |
| One-way latency (time of transmission to time of reception) | Firmware measures round-trip time with ping and the packet test |

These are open items for future firmware builds.

---

## Authors

- **Hilario, Steffi Anne M.**
- **Paneda, Christian Aron R.**
- **Rivera, Pauleen Ann C.**
- **Tabelisma, Sean Martin Andrei V.**

**Thesis Adviser:** Engr. Moises F. Jardiniano

Bachelor of Science in Computer Engineering
FEU Institute of Technology, 2026

---

## References

Selected references from the thesis:

1. Muladi et al., "LoRa Mesh-Based IoT GPS Tracking System for Mountain Climbers," *International Journal of Safety and Security Engineering*, vol. 14, no. 6, 2024. doi:10.18280/ijsse.140610
2. B. Li et al., "LoRaWAPS: A Wide-Area Positioning System Based on LoRa Mesh," *Applied Sciences*, vol. 13, no. 17, 2023. doi:10.3390/app13179501
3. A. Luinge and S. N. Anbalagan, "Evaluating LoRa Mesh Networks for Personal Dead Reckoning of Firefighters," *Lecture Notes in Computer Science*, 2026. doi:10.1007/978-3-032-13312-0_22
4. M. A. M. Almuhaya et al., "A Survey on LoRaWAN Technology," *Electronics*, vol. 11, no. 1, 2022. doi:10.3390/electronics11010164
5. A. I. L. Cadeliña et al., "LoRaWAN-based Anti-Kidnapping Device with Built-in GPS Navigator and Software Application Tracker," IEEE ICEIB, 2024. doi:10.1109/iceib61477.2024.10602616

See the full thesis for the complete reference list.

---

## License

No license has been specified yet. Please contact the authors before reusing this work.
