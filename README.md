<div align="center">

<img src="https://img.shields.io/badge/ESP32-ESP--NOW-red?style=for-the-badge&logo=espressif&logoColor=white"/>
<img src="https://img.shields.io/badge/Status-Active%20Development-brightgreen?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Protocol-ESP--NOW-blue?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Platform-ESP32%20%7C%20Arduino%20Nano-orange?style=for-the-badge&logo=arduino"/>

# 📡 Universal RC Controller — URC

**A fully custom wireless RC controller system built from scratch.**  
Hand-wired hardware · ESP-NOW radio link · Modular receiver library · Multi-project compatible

**by [Aniket Chowdhury](https://github.com/itzzhashtag) (aka `#Hashtag`)**

</div>

---

## 🧭 What is URC?

**Universal RC (URC)** is a DIY wireless controller platform built around ESP32 and the **ESP-NOW** protocol. It started as a simple joystick-to-robot link and has grown into a full-featured RC system with safety gates, battery monitoring, OLED/LCD display, UART display offloading, rich audio feedback, and a plug-and-play receiver library.

The goal: **one controller to rule them all** — hexapods, bipeds, race cars, LED games, anything.

---

## 📂 Version History

| Version | Highlights | Status |
|---------|-----------|--------|
| [v0.4 – v0.9](./versions/v0.4-v0.9/) | Basic joystick + ESP-NOW link, single-file sketch | Archive |
| [v1.0](./versions/v1.0/) | Added toggle switches, pots, battery ADC | Archive |
| [v1.1](./versions/v1.1/) | ACK-based connection watchdog, LED indicators | Archive |
| [v1.2](./versions/v1.2/) | ST7920 SPI LCD via U8g2, non-blocking buzzer | Archive |
| [v1.5](./versions/v1.5/) | ReceiverModule.h singleton class introduced | Archive |
| [v1.9](./versions/v1.9/) | Full display screens on ESP32, sound engine | Archive |
| **[v2.0](./v2.0/)** | **Nano display offload · 18-byte UART packet · 4-screen UI · Radiomaster-style audio** | ✅ **Current** |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────┐        ┌────────────────────────────────────┐
│         ESP32 Transmitter           │        │        ESP32 Receiver              │
│                                     │        │                                    │
│  2× Joystick  2× Toggle  2× Pot    │        │  ReceiverModule.h                  │
│  4× Button    ADS1115 Bat  Buzzer  │        │  receiver.begin()                  │
│                                     │  ESP   │  receiver.update()                 │
│  ControllerData struct (25 bytes)  │ ──NOW──▶│  receiver.available()              │
│  ACK watchdog ← AckData struct     │◀───────│                                    │
│                                     │        │  → Your robot / servo / motor code │
└──────────────┬──────────────────────┘        └────────────────────────────────────┘
               │ UART Serial2 (18-byte packet)
               ▼
┌──────────────────────────────┐
│      Arduino Nano            │
│  JHD12864E KS0108 128×64    │
│  4 screens: Boot / Caution / │
│  LowBat / Normal             │
└──────────────────────────────┘
```

---

## 🎮 Controller Hardware

| Input | Count | Range |
|-------|-------|-------|
| Analog Joystick Axes | 4 (LX/LY/RX/RY) | −99 … +99 |
| Shoulder Buttons | 2 (LBt/RBt) | bool |
| Joystick Clicks | 2 (LABt/RABt) | bool |
| Toggle Switches | 2 (TSW1/TSW2) | bool |
| Slider Potentiometers | 2 (Pot1/Pot2) | 0 … 100 |
| Battery Monitor | ADS1115 I²C | 0 … 100 % |

---

## 📡 ReceiverModule.h — Drop-in Library

Copy `ReceiverModule.h` into any project and you're done.

```cpp
#include "ReceiverModule.h"

void setup() {
  receiver.begin();                    // broadcast — accept any TX
  // receiver.begin("AA:BB:CC:DD:EE:FF"); // unicast — specific TX only
}

void loop() {
  receiver.update();                   // MUST call — handles failsafe
  if (receiver.available()) {
    int speed = receiver.data.Ly;      // left stick Y
    bool arm  = receiver.data.TSW1;   // toggle switch 1
  }
}
```

**Features:** MAC filtering · auto ACK reply · 1-second failsafe data-zero · ESP32 Core 2.x/3.x compatible

---

## 🤖 Projects Using URC

| Project | Receiver Hardware | Notes |
|---------|-----------------|-------|
| ESP32 Hexapod | 2× PCA9685 · 18× MG Servo | Tripod gait · body sway · IK |
| Otto Biped (SiBot) | Arduino + Servo lib | Safe angle limits · 2S LiPo |
| 1D LED Racing Game | ESP32 + NeoPixel | 4-player · ESP-NOW buttons |
| 1D LED Pong | Arduino Uno + WS2812B | Physics engine · TM1637 display |

---

## 🗂️ Repository Structure

```
URC/
├── README.md                  ← You are here
├── ReceiverModule.h           ← Universal receiver library
├── v2.0/
│   ├── README.md              ← v2.0 detailed docs
│   ├── ESP32_TX_v2.0/        ← Transmitter firmware
│   └── Nano_Display_v2.0/    ← KS0108 display firmware
└── versions/
    └── v0.4 … v1.9/          ← Archived versions
```

---

## 📋 Requirements

**Transmitter:** ESP32 · Adafruit ADS1X15 library  
**Display Nano:** Arduino Nano · U8g2 library (`U8G2_KS0108_128X64_F`)  
**Receiver:** Any ESP32 · no extra libraries (just copy `ReceiverModule.h`)

---

<div align="center">

**[📖 v2.0 Detailed Docs](./v2.0/README.md)** · **[📧 Contact](mailto:micro.aniket@gmail.com)** · **[💼 LinkedIn](https://linkedin.com/in/itzz-hashtag)** · **[🐙 GitHub](https://github.com/itzzhashtag)**

⭐ Star this repo if URC saved you from building your own from scratch!

</div>
