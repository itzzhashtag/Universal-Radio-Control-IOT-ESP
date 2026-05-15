<div align="center">

# 📡 URC v2.0 — Universal RC Controller

**What's new:** Display offloaded to Arduino Nano · 4-screen KS0108 UI · 18-byte UART protocol · Radiomaster-style sweep audio

**by [Aniket Chowdhury](https://github.com/itzzhashtag) (aka `#Hashtag`)**  

<img src="https://img.shields.io/badge/URC-v2.0-blueviolet?style=for-the-badge&logo=espressif"/>
<img src="https://img.shields.io/badge/ESP--NOW-Wireless-red?style=for-the-badge&logo=wifi"/>
<img src="https://img.shields.io/badge/Display-KS0108%20128×64-cyan?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Audio-Radiomaster%20Style-orange?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Status-Stable-brightgreen?style=for-the-badge"/>



</div>

---

## 🔑 v2.0 vs v1.9 — What Changed

| Feature | v1.9 | v2.0 |
|---------|------|------|
| Display driver | U8g2 running on ESP32 | Offloaded to Arduino Nano |
| UART protocol | None | Structured 18-byte packet + XOR checksum |
| Screen count | 1 (normal only) | 4 screens (boot / caution / lowbat / normal) |
| Audio | Single `tone()` beep | Full sweep-based sound profiles |
| Boot settle | Blocking `delay()` | Animated `SCREEN_BOOT` stream to Nano |
| Serial debug | Verbose spam | Clean `[TAG]` prefixed lines only |

---

## 📸 Build Photos

<div align="center">

<!-- 🖼️ Replace src with your actual build photos -->

| Controller (Front) | Controller (Back) | Internals |
|:-----------------:|:-----------------:|:---------:|
| <img src="./assets/v2_front.jpg" width="220" alt="Front"/> | <img src="./assets/v2_back.jpg" width="220" alt="Back"/> | <img src="./assets/v2_inside.jpg" width="220" alt="Inside"/> |

</div>

---

## 🏗️ System Architecture

```
 ┌─────────────────────────────────────────────────────────┐
 │                ESP32 Transmitter (TX)                   │
 │                                                         │
 │  LJoy(35,32)  RJoy(33,34)  Btns(26,27,25,13)           │
 │  Toggle(18,19)  Pot(39,36)  ADS1115-I2C  Buzzer(23)    │
 │                        ↓                                │
 │          ControllerData struct → esp_now_send()         │
 │                                        ↑ AckData        │
 │  Serial2 GPIO17→ ─────────────────────────────────────► │
 │  18-byte UART packet @ LCD_FPS Hz                       │
 └─────────────────────────────────────────────────────────┘
                          │ UART 115200 baud
                          ▼
 ┌─────────────────────────────────────────────────────────┐
 │              Arduino Nano — Display Slave               │
 │  Parses packet → XOR check → unpacks → renders          │
 │  JHD12864E · KS0108 · 128×64 px · U8g2 full-buffer     │
 └─────────────────────────────────────────────────────────┘
                                       │ ESP-NOW (broadcast/unicast)
                                       ▼
 ┌─────────────────────────────────────────────────────────┐
 │              ESP32 Receiver (any robot/vehicle)         │
 │  #include "ReceiverModule.h"                            │
 │  receiver.begin() · receiver.update() · receiver.data   │
 └─────────────────────────────────────────────────────────┘
```

---

## 📦 Files

| File | Role |
|------|------|
| `ESP32_TX_v2.0.ino` | Transmitter — inputs, ESP-NOW send, UART to Nano |
| `Nano_Display_v2.0.ino` | Nano slave — UART parser, KS0108 renderer |
| `ReceiverModule.h` | Drop-in receiver library (copy into any robot sketch) |
| `URC_Example_RX.ino` | Minimal example using ReceiverModule |
| `LCD_Visualizer.html` | Browser-based live LCD preview (interactive) |

---

## 🔌 Wiring

### ESP32 Transmitter Pin Map

| Component | GPIO | Notes |
|-----------|:----:|-------|
| Left Joystick X | 35 | ADC1 input-only |
| Left Joystick Y | 32 | ADC1 |
| Right Joystick X | 33 | ADC1 |
| Right Joystick Y | 34 | ADC1 input-only |
| L Stick Click (LABt) | 26 | INPUT_PULLUP · active LOW |
| R Stick Click (RABt) | 27 | INPUT_PULLUP · active LOW |
| Left Shoulder (LBt) | 25 | INPUT_PULLUP · active LOW |
| Right Shoulder (RBt) | 13 | INPUT_PULLUP · active LOW |
| Toggle Switch 1 | 18 | INPUT_PULLUP |
| Toggle Switch 2 | 19 | INPUT_PULLUP |
| Pot 1 | 39 | ADC1 input-only |
| Pot 2 | 36 | ADC1 input-only |
| ADS1115 SDA / SCL | 21 / 22 | I²C battery monitor |
| Red LED | 2 | 100Ω → GND |
| Green LED | 4 | 100Ω → GND |
| Passive Buzzer | 23 | 100Ω → GND |
| UART TX2 → Nano RX | 17 | Serial2 |
| UART RX2 ← Nano TX | 16 | Optional |

> ⚠️ **ADC1 only (GPIO 32–39).** ADC2 shares silicon with the WiFi radio — using it causes jitter and crashes during ESP-NOW transmit.

### Arduino Nano ↔ KS0108 (JHD12864E)

| LCD Pin | Nano Pin | Notes |
|---------|:--------:|-------|
| D0 – D7 | 2 – 9 | 8-bit parallel data bus |
| EN | A4 (18) | Enable strobe |
| RS / DC | A5 (19) | Register select |
| CS1 | 10 | Left half — pixels 0–63 |
| CS2 | 11 | Right half — pixels 64–127 |
| **R/W** | **GND** | ⚠️ Tie directly to GND — write-only |
| RST | — | Not connected (`U8X8_PIN_NONE`) |
| VCC / GND | 5V / GND | — |

> ⚠️ **Disconnect Nano pin 0 (RX) before uploading.** The UART from ESP32 conflicts with the USB programmer.

### Wiring Diagram

<div align="center">

<!-- 🖼️ Add your Fritzing / hand-drawn / KiCad wiring diagram here -->

| ESP32 TX Wiring | Nano ↔ LCD Wiring |
|:---------------:|:-----------------:|
| <img src="./assets/wiring_esp32.png" width="320" alt="ESP32 Wiring Diagram"/> | <img src="./assets/wiring_nano_lcd.png" width="320" alt="Nano LCD Wiring"/> |

<!-- 🖼️ Full schematic (optional) -->
<!-- <img src="./assets/schematic_v2.png" width="700" alt="Full Schematic"/> -->

</div>

---

## 📟 UART Packet — 18 Bytes

```
[0]  0xAA          — Start marker (never checksummed)
[1]  screenID      — 0x01=BOOT  0x02=CAUTION  0x03=LOWBAT  0x04=NORMAL
[2]  cautionFlags  — bit0=SW1_OK  bit1=SW2_OK  bit2=P1_OK  bit3=P2_OK  bit4=BAT_OK
[3]  Lx + 99       — Left  stick X  (0–198, signed offset)
[4]  Ly + 99       — Left  stick Y
[5]  Rx + 99       — Right stick X
[6]  Ry + 99       — Right stick Y
[7]  Pot1           — 0–100
[8]  Pot2           — 0–100
[9]  BAT %          — 0–100
[10] Button mask    — bit0=LABt  bit1=RABt  bit2=LBt  bit3=RBt
                      bit4=TSW1  bit5=TSW2  bit6=gConnected
[11–16] Reserved   — 0x00
[17] XOR checksum  — XOR of bytes [1]..[16]
```

Nano validates checksum before rendering. Bad/partial packets are silently dropped — last good frame holds on screen.

---

## 🖥️ Display Screens

<div align="center">

<!-- 🖼️ Screenshot each screen from the LCD_Visualizer.html and drop them here -->

| Screen | Preview | Description |
|--------|:-------:|-------------|
| 🟡 **BOOT** | <img src="./assets/screen_boot.png" width="200" alt="Boot Screen"/> | Bouncing box animation while ADS1115 voltage settles (4 s default) |
| 🔴 **CAUTION** | <img src="./assets/screen_caution.png" width="200" alt="Caution Screen"/> | Live per-check gate: SW1 / SW2 / P1 / P2 / BAT must all show OK |
| 🔋 **LOWBAT** | <img src="./assets/screen_lowbat.png" width="200" alt="Low Battery Screen"/> | Radio killed · outputs zeroed · large battery icon blinks |
| 🟢 **NORMAL** | <img src="./assets/screen_normal.png" width="200" alt="Normal HUD"/> | Full live HUD: joysticks · pots · signal bars · buttons · battery |

</div>

### Normal HUD Layout (128×64)

```
y  0–28 │ [L-Joy]  [P1▐]  ·····  [▮▮▮▮ Signal]  ·····  [P2▐]  [R-Joy] │
y    29 ├──────────────────────────── divider ───────────────────────────┤
y 30–44 │   [LB]  [LS]  [S1]                 [S2]  [RS]  [RB]           │
y    45 ├──────────────────────────── divider ───────────────────────────┤
y 46–63 │ ┌─ HASHTAG ─┐                          [🔋▌▌▌▌]  82%         │
        │ │  URC V2.0  │                                                  │
        │ └────────────┘                                                  │
```

---

## 🎮 Live LCD Visualizer

An interactive browser-based preview that renders the **exact same pixel layout** as the real KS0108 firmware — useful for testing layout changes before flashing.

<div align="center">

<!-- 🖼️ Screenshot of the HTML visualizer running in your browser -->
<img src="./assets/visualizer_screenshot.png" width="500" alt="LCD Visualizer Screenshot"/>

</div>

> 📄 **Open [`LCD_Visualizer.html`](./LCD_Visualizer.html)** in any browser.  
> Drag the sliders and tick the checkboxes to see joysticks, pots, signal bars, buttons, and battery update live — no hardware needed.

**What it simulates:**
- Live joystick dot position tracking inside crosshair box
- 4-bar animated signal widget (fill cycle when connected, blinking ✕ when not)
- Vertical pot sliders
- Button invert-on-press (white text on black)
- Battery fill + low-battery blink at < 20 %
- All at 2× pixel zoom with authentic green-on-black LCD palette

---

## 🔊 Sound Engine

Sweep-based audio using rapid `tone()` steps — simulates Radiomaster TX16S-style sounds on a passive buzzer.

| Event | Profile | Feel |
|-------|---------|------|
| Boot complete | `toneBoot()` | 3-stage rising sweep → sustain pip |
| Link acquired | `toneConnected()` | Fast whoop → two confident bips |
| Link lost | `toneDisconnected()` | Alert pip → falling wail → low thuds |
| Low battery | `toneLowBat()` | Three urgent descending stabs |
| Joystick click | `soundJoyClick()` | Micro rising sweep — light "tik" |
| Shoulder button | `soundShoulderBtn()` | Low thunk + rising confirm pip |
| Toggle ON / OFF | `soundToggleOn/Off()` | Ascending / descending blip |
| Pot movement | `soundPotTick()` | Chirp pitched to pot position |

`softStop()` tails off to ~80 Hz before `noTone()` to avoid the audible click from abrupt square-wave cutoff.

---

## 🔋 Battery System

| Parameter | Value |
|-----------|-------|
| Sensor | ADS1115 · GAIN_TWOTHIRDS · ±6.144 V · 0.1875 mV/LSB |
| Voltage divider | 100 kΩ + 47 kΩ → ratio 3.128× |
| Pack type | 2S Li-Ion |
| Full / Empty | 8.4 V (100 %) / 6.6 V (0 %) |
| Safe mode trigger | ≤ 6.8 V → radio killed · outputs zeroed |
| Startup block | < 10 % → CAUTION gate holds |
| Calibration | `BAT_CAL_FACTOR` trim multiplier |

---

## ⚙️ Tunable Constants

```cpp
// ── In ESP32_TX_v2.0.ino ─────────────────────────────────────
#define LCD_FPS         60    // Loop + UART packet rate (Hz)
#define BOOT_SETTLE_MS  4000  // Boot animation duration (ms)
#define DEADZONE        15    // Joystick centre dead-band
#define CHANGE_THRESH   2     // Min axis delta before updating txData
#define POT_ZERO_THRESH 80    // Raw ADC ≤ this → pot at minimum
#define ACK_TIMEOUT     500   // ms without ACK → disconnected
#define BAT_LOW_V       6.8f  // V → safe mode threshold
#define BROADCAST       1     // 1 = FF:FF:… broadcast  |  0 = unicast
```

---

## 🚀 Getting Started

**1. Flash Transmitter**  
Open `ESP32_TX_v2.0.ino`, flash to ESP32. Serial monitor at 115200 shows the TX MAC address at boot.

**2. Flash Display Nano**  
⚠️ Disconnect Nano pin 0 (RX) first. Flash `Nano_Display_v2.0.ino`. Reconnect pin 0 after.

**3. Wire UART Link**  
`ESP32 GPIO17` → `Nano RX0` · `GND` shared · LCD `R/W` → `GND`.

**4. Set Up Receiver**  
Copy `ReceiverModule.h` into your robot sketch folder. Paste TX MAC from step 1 into `receiver.begin("XX:XX:XX:XX:XX:XX")`.

**5. Power On & Check Sequence**  
`SCREEN_BOOT` (4 s) → `SCREEN_CAUTION` (set SW1/SW2 OFF, pots to minimum) → `SCREEN_NORMAL` + boot tone.

---

## 📚 Dependencies

| Side | Library | Install |
|------|---------|---------|
| ESP32 TX | Adafruit ADS1X15 | Library Manager |
| Arduino Nano | U8g2 by olikraus | Library Manager |
| ESP32 RX | None | Just copy `ReceiverModule.h` |

---

## 👤 Author & Contact

👨 **Name:** Aniket Chowdhury (aka Hashtag)  
📧 **Email:** [micro.aniket@gmail.com](mailto:micro.aniket@gmail.com)  
💼 **LinkedIn:** [itzz-hashtag](https://www.linkedin.com/in/itzz-hashtag/)  
🐙 **GitHub:** [itzzhashtag](https://github.com/itzzhashtag)  
📸 **Instagram:** [@itzz_hashtag](https://instagram.com/itzz_hashtag)

---

## 📜 License

This project is released under a Modified MIT License.
It is intended for personal and non-commercial use only.

🚫 Commercial use or distribution for profit is not permitted without prior written permission.
🤝 For collaboration, reuse, or licensing inquiries, please contact the author.

📄 View Full License <br>
[![License: MIT–NC](https://img.shields.io/badge/license-MIT--NC-blue.svg)](./LICENSE)

---

## ❤️ Acknowledgements

This is a solo passion project, built with countless nights of tinkering, testing, and debugging.  
If you find it useful or inspiring, feel free to ⭐ the repository or connect with me on social media!

---

> _“If the mind can create, the hands can translate.”_ – Hashtag
