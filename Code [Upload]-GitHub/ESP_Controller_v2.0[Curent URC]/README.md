<div align="center">

<img src="https://img.shields.io/badge/URC-v2.0-blueviolet?style=for-the-badge&logo=espressif"/>
<img src="https://img.shields.io/badge/ESP--NOW-Wireless-red?style=for-the-badge&logo=wifi"/>
<img src="https://img.shields.io/badge/Display-KS0108%20128×64-cyan?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Audio-Radiomaster%20Style-orange?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Status-Stable-brightgreen?style=for-the-badge"/>

# 📡 URC v2.0 — Universal RC Controller

> **What's new:** Display offloaded to Arduino Nano · 4-screen KS0108 UI · 18-byte UART protocol · Radiomaster-style sound engine

**by [Aniket Chowdhury](https://github.com/itzzhashtag) (aka `#Hashtag`)**

[← Back to main README](../README.md)

</div>

---

## 🔑 What Changed in v2.0

| Feature | Before (v1.9) | v2.0 |
|---------|--------------|------|
| Display driver | U8g2 on ESP32 | Offloaded to Arduino Nano |
| UART protocol | None | Structured 18-byte packet |
| Screen count | 1 (normal) | 4 screens (boot/caution/lowbat/normal) |
| Audio | Single beep | Full sweep-based sound engine |
| Serial debug | Verbose spam | Clean tagged lines only |
| Boot settle | Blocking delay | Animated SCREEN_BOOT streaming |

---

## 🏗️ System Overview

```
 ┌──────────────────────────────────────────────────────────────────┐
 │                   ESP32 Transmitter (TX)                         │
 │                                                                  │
 │  [LJoy]  [RJoy]  [LBt][RBt]  [TSW1][TSW2]  [Pot1][Pot2]       │
 │     ↓       ↓        ↓             ↓              ↓             │
 │           ControllerData struct (25 bytes)                       │
 │                        ↓                                         │
 │              esp_now_send() → peerMAC                           │
 │                                        ← AckData (alive=true)   │
 │                                                                  │
 │  Serial2 (GPIO17 TX → Nano RX0)                                  │
 │  18-byte UART packet every frame  ──────────────────────────────►│
 └──────────────────────────────────────────────────────────────────┘
                                     ▼
 ┌──────────────────────────────────────────────────────────────────┐
 │                   Arduino Nano (Display)                         │
 │                                                                  │
 │  Parses 18-byte packet → unpacks to dScreen, dLx, dLy…          │
 │  Renders correct screen on JHD12864E (KS0108 128×64)            │
 │    SCREEN_BOOT    → Bouncing box + "Booting…"                   │
 │    SCREEN_CAUTION → Live SW/POT/BAT gate status                 │
 │    SCREEN_LOWBAT  → Large blinking bat icon                     │
 │    SCREEN_NORMAL  → Full controller HUD                         │
 └──────────────────────────────────────────────────────────────────┘
```

---

## 📦 Files in This Version

| File | Role |
|------|------|
| `ESP32_TX_v2.0.ino` | Transmitter — reads inputs, sends ESP-NOW, drives UART |
| `Nano_Display_v2.0.ino` | Nano slave — receives UART packets, renders KS0108 |
| `ReceiverModule.h` | Drop-in receiver library for robot/vehicle side |
| `URC_Example_RX.ino` | Example receiver sketch using ReceiverModule.h |

---

## 🔌 Hardware & Wiring

### ESP32 Transmitter

| Component | GPIO | Notes |
|-----------|------|-------|
| Left Joystick X | 35 | ADC1 input-only |
| Left Joystick Y | 32 | ADC1 |
| Right Joystick X | 33 | ADC1 |
| Right Joystick Y | 34 | ADC1 input-only |
| LABt (L stick click) | 26 | INPUT_PULLUP, active LOW |
| RABt (R stick click) | 27 | INPUT_PULLUP, active LOW |
| LBt (left shoulder) | 25 | INPUT_PULLUP, active LOW |
| RBt (right shoulder) | 13 | INPUT_PULLUP, active LOW |
| Toggle Switch 1 | 18 | INPUT_PULLUP |
| Toggle Switch 2 | 19 | INPUT_PULLUP |
| Pot 1 | 39 | ADC1 input-only |
| Pot 2 | 36 | ADC1 input-only |
| ADS1115 SDA/SCL | 21/22 | I²C battery monitor |
| Red LED | 2 | 100Ω → GND |
| Green LED | 4 | 100Ω → GND |
| Passive Buzzer | 23 | 100Ω → GND |
| UART TX2 to Nano | 17 | → Nano RX0 |
| UART RX2 from Nano | 16 | ← Nano TX1 (optional) |

> ⚠️ Use **ADC1 pins only** (32–39). ADC2 conflicts with WiFi/ESP-NOW.

### Arduino Nano — KS0108 Display

| LCD Pin | Nano Pin | Notes |
|---------|---------|-------|
| D0–D7 | 2–9 | 8-bit parallel data bus |
| EN | A4 (18) | Enable |
| RS/DC | A5 (19) | Register select |
| CS1 | 10 | Left half (pixels 0–63) |
| CS2 | 11 | Right half (pixels 64–127) |
| R/W | **GND** | Write-only — tie to GND |
| RST | — | Not connected (U8X8_PIN_NONE) |
| VCC | 5V | — |

> ⚠️ **Disconnect Nano RX (pin 0) before uploading sketches** — UART from ESP32 conflicts with the programmer.

---

## 📟 UART Packet Protocol (18 bytes)

```
Byte  0   : 0xAA       ← Start marker
Byte  1   : screenID   ← 0x01=BOOT  0x02=CAUTION  0x03=LOWBAT  0x04=NORMAL
Byte  2   : cautionFlags (bitmask)
              bit0=SW1_OK  bit1=SW2_OK  bit2=P1_OK  bit3=P2_OK  bit4=BAT_OK
Byte  3   : Lx + 99    ← Left  stick X  (0..198, offset to avoid signed bytes)
Byte  4   : Ly + 99    ← Left  stick Y
Byte  5   : Rx + 99    ← Right stick X
Byte  6   : Ry + 99    ← Right stick Y
Byte  7   : Pot1       ← 0..100
Byte  8   : Pot2       ← 0..100
Byte  9   : BAT %      ← 0..100
Byte 10   : Button bitmask
              bit0=LABt  bit1=RABt  bit2=LBt  bit3=RBt
              bit4=TSW1  bit5=TSW2  bit6=gConnected
Bytes 11–16 : Reserved (0x00)
Byte 17   : XOR checksum of bytes [1]..[16]
```

Nano validates checksum before rendering. Bad packets are silently dropped — last good state holds on screen.

---

## 🖥️ Display Screens

### `SCREEN_BOOT` — Capacitor Settle Animation
Shown during `BOOT_SETTLE_MS` (default 4000 ms) while the ADS1115 voltage stabilises.  
A filled 8×8 box bounces left↔right across the bottom of the display.

### `SCREEN_CAUTION` — Startup Safety Gate
Blocks until **all** conditions are met simultaneously:

| Check | Condition | Why |
|-------|-----------|-----|
| SW1 | Must be OFF (HIGH) | Prevent robot lurching on power-up |
| SW2 | Must be OFF (HIGH) | — |
| Pot1 | Must be at minimum | Avoid jump-to-position on startup |
| Pot2 | Must be at minimum | — |
| Battery | ≥ 10 % | Don't operate on a dying pack |

Live status shown per-check as `OK` / `HIGH!` / `LOW!`.

### `SCREEN_LOWBAT` — Safe Mode
Triggered when `batV ≤ BAT_LOW_V` (default 6.8 V).  
ESP-NOW radio killed · all outputs zeroed · large battery icon blinks.  
Recovery re-runs the full startup gate.

### `SCREEN_NORMAL` — HUD

```
┌────────────────────────────────────────────────────────────────┐
│ [L-Joy]  [P1▐]  ≡≡≡≡  [Signal bars]  ≡≡≡≡  [P2▐]  [R-Joy]   │  y 0–28
├────────────────────────────────────────────────────────────────┤  y 29
│  [LB] [LS] [S1]              [S2] [RS] [RB]                   │  y 30–44
├────────────────────────────────────────────────────────────────┤  y 45
│ ┌──────────────┐                    🔋▌▌▌▌ 87%               │  y 46–63
│ │ HASHTAG      │                                              │
│ │ URC V2.0     │                                              │
│ └──────────────┘                                              │
└────────────────────────────────────────────────────────────────┘
```

- Joystick dots track live position inside crosshair box
- 4-bar animated signal widget (fills on connect, blinks ✕ on loss)
- Vertical slider bars for both pots
- 6 button widgets invert on press (white text on black)
- Battery icon fills/blinks from right (positive terminal right)

---

## 🔊 Sound Engine

All sounds use rapid `tone()` frequency sweeps — no blocking `delay()` between steps. Inspired by Radiomaster TX16S audio profiles.

| Event | Sound | Description |
|-------|-------|-------------|
| Boot complete | `toneBoot()` | 3-stage rising sweep → sustain pip |
| Link acquired | `toneConnected()` | Fast whooooop → two confident bips |
| Link lost | `toneDisconnected()` | Alert pip → falling wail → low thuds |
| Low battery | `toneLowBat()` | Three descending stabs |
| Joystick click | `soundJoyClick()` | Micro rising sweep (light "tik") |
| Shoulder button | `soundShoulderBtn()` | Low thunk + rising pip |
| Toggle ON | `soundToggleOn()` | Ascending blip |
| Toggle OFF | `soundToggleOff()` | Descending blip |
| Pot movement | `soundPotTick()` | Chirp pitched to pot position |

`softStop()` is used at the end of sounds that end at low frequencies — prevents the audible "pop" from abrupt square-wave cutoff at low Hz.

---

## 🔋 Battery System

- **Sensor:** ADS1115 at GAIN_TWOTHIRDS (±6.144 V, 0.1875 mV/LSB)
- **Divider:** 100 kΩ + 47 kΩ → ratio `147/47 = 3.128`
- **Pack:** 2S Li-Ion, 6.6 V (0 %) → 8.4 V (100 %)
- **Safe mode threshold:** 6.8 V
- **Startup block threshold:** 10 %
- **Calibration:** `BAT_CAL_FACTOR` trim constant for divider error

---

## ⚙️ Tunable Constants (top of ESP32 firmware)

```cpp
#define LCD_FPS         60    // Loop rate in Hz (also sets UART packet rate)
#define BOOT_SETTLE_MS  4000  // Boot animation duration (ms)
#define DEADZONE        15    // Joystick centre dead-band (mapped units)
#define CHANGE_THRESH   2     // Min axis delta to update txData (kills jitter)
#define POT_ZERO_THRESH 80    // Raw ADC ≤ this = "pot at minimum" for gate
#define ACK_TIMEOUT     500   // ms without ACK → mark disconnected
#define BAT_LOW_V       6.8f  // V → triggers safe mode
#define BROADCAST       1     // 1=broadcast  0=unicast to ROBOT_MAC_BYTES
```

---

## 📡 ESP-NOW Link Details

- **Mode:** Broadcast by default (`BROADCAST 1`) — any receiver hears packets
- **Switch to unicast:** Set `BROADCAST 0` and paste target MAC in `ROBOT_MAC_BYTES`
- **Connection detection:** Receiver sends `AckData{alive=true}` back; TX watchdog checks within `ACK_TIMEOUT` ms
- **Core compatibility:** Callback uses `esp_now_recv_info_t*` (Core 3.x); Core 2.x note included in comments

---

## 📚 Dependencies

| Library | Used In | Install Via |
|---------|---------|-------------|
| Adafruit ADS1X15 | ESP32 TX | Library Manager |
| U8g2 (olikraus) | Arduino Nano | Library Manager |
| ESP-NOW | ESP32 TX + RX | Built-in (ESP32 core) |

---

## 🚀 Getting Started

1. **Transmitter** — Flash `ESP32_TX_v2.0.ino` to your ESP32. Open Serial monitor at 115200 to see boot MAC address.
2. **Display** — Flash `Nano_Display_v2.0.ino` to your Arduino Nano. **Disconnect pin 0 first.**
3. **Receiver** — Copy `ReceiverModule.h` into your robot sketch folder. Call `receiver.begin()` in setup.
4. **Wiring** — ESP32 GPIO17 → Nano RX0 · share GND · tie LCD R/W to GND.
5. **Power on** — Observe SCREEN_BOOT (4 s) → SCREEN_CAUTION (put switches OFF, pots to minimum) → SCREEN_NORMAL.

---

<div align="center">

## 👤 Author

**Aniket Chowdhury (aka `#Hashtag`)**

[![Email](https://img.shields.io/badge/Email-micro.aniket@gmail.com-red?style=flat-square&logo=gmail)](mailto:micro.aniket@gmail.com)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-itzz--hashtag-blue?style=flat-square&logo=linkedin)](https://linkedin.com/in/itzz-hashtag)
[![GitHub](https://img.shields.io/badge/GitHub-itzzhashtag-black?style=flat-square&logo=github)](https://github.com/itzzhashtag)
[![Instagram](https://img.shields.io/badge/Instagram-itzz__hashtag-purple?style=flat-square&logo=instagram)](https://instagram.com/itzz_hashtag)

[← Back to main README](../README.md)

⭐ If URC saved you time, leave a star!

</div>
