<div align="center">

# 📡 Universal RC Controller — URC

**A fully custom wireless RC system built from scratch.**  
Hand-wired hardware · ESP-NOW radio link · Modular receiver library · Multi-project compatible

**by [Aniket Chowdhury](https://github.com/itzzhashtag) (aka `#Hashtag`)**

<img src="https://img.shields.io/badge/ESP32-ESP--NOW-red?style=for-the-badge&logo=espressif&logoColor=white"/>
<img src="https://img.shields.io/badge/Status-Active%20Development-brightgreen?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Protocol-ESP--NOW-blue?style=for-the-badge"/>
<img src="https://img.shields.io/badge/Platform-ESP32%20%7C%20Arduino%20Nano-orange?style=for-the-badge&logo=arduino"/>
<img src="https://img.shields.io/badge/Current-v2.0-blueviolet?style=for-the-badge"/>

</div>

---

## 🧭 What is URC?

**Universal RC (URC)** is a DIY wireless controller platform built around the ESP32 and **ESP-NOW** protocol. One hand-built controller that talks to anything — hexapods, bipeds, LED games, RC cars. Features safety gates, battery monitoring, KS0108 LCD, UART display offloading, and a Radiomaster-style audio engine.

> One controller to rule them all.

---

## 📸 Controller Photos

<div align="center">

<!-- 🖼️ Replace the src paths with your actual photos. Recommended: front, internals, in-use -->

| Front View of V1.0 | Front View of V2.0 | In Action | Caution Mode |
|:----------:|:---------:|:---------:|:---------:|
| <img src="https://github.com/user-attachments/assets/0d5c9cc6-242d-417b-882d-dd49d86de769" width="250" alt="Front View of V1.0"/> | <img src="https://github.com/user-attachments/assets/790e8f8d-c663-4fce-bc0d-e49fa5c03b9e" width="250" alt="Front View of V2.0"/> | <img src="https://github.com/user-attachments/assets/b348c5ab-b535-49fc-99af-768a9f4a50ab" width="250" alt="In Action"/> | <img src="https://github.com/user-attachments/assets/6eb9c748-f37b-481a-adf7-f7afdfce545c" width="250" alt="Caution Mode"/> |

</div>

---

## 📸 Wiring Schematics

<div align="center">

<!-- 🖼️ Replace the src paths with your actual photos. Recommended: front, internals, in-use -->

| Breadboard | Schematics |
|:----------:|:---------------:|
| <img src="https://github.com/user-attachments/assets/003fb99e-7e1f-437a-b3b6-bc4aece9a538" width="500" height="500" alt="Breadboard"/> | <img src="https://github.com/user-attachments/assets/4b284221-0378-461c-9655-05f1df40999f" width="500" height="500" alt="Schematics"/>
</div>

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────┐       ┌──────────────────────────────────┐
│          ESP32 Transmitter (TX)          │       │      ESP32 Receiver (RX)         │
│                                          │       │                                  │
│  2× Joystick · 2× Toggle · 2× Pot       │       │  #include "ReceiverModule.h"     │
│  4× Button · ADS1115 Bat · Buzzer       │ ESP   │  receiver.begin()                │
│  RED/GREEN LEDs                          │─NOW──▶│  receiver.update()               │
│                                          │◀──────│  receiver.available()            │
│  ControllerData → esp_now_send()         │  ACK  │                                  │
└──────────────┬───────────────────────────┘       │  → Your robot / motors / servos  │
               │ UART 115200 · 18-byte packet       └──────────────────────────────────┘
               ▼
┌──────────────────────────────────────┐
│         Arduino Nano (Display)       │
│  JHD12864E · KS0108 · 128×64 px     │
│  BOOT → CAUTION → LOWBAT → NORMAL   │
└──────────────────────────────────────┘
```

---

## 🖥️ Display Screens

<div align="center">

| 🟡 Boot | 🔴 Caution Gate | 🔋 Low Battery | 🟢 Normal HUD |
|:-------:|:---------------:|:--------------:|:-------------:|
| Bouncing box while caps settle | Live SW/POT/BAT gate status | Radio killed · outputs zeroed | Full controller HUD |

</div>

---

## 🎮 Inputs at a Glance

| Input | Qty | Range |
|-------|:---:|:-----:|
| Analog Joystick Axes | 4 (LX/LY/RX/RY) | −99 … +99 |
| Shoulder Buttons | 2 (LBt/RBt) | bool |
| Joystick Clicks | 2 (LABt/RABt) | bool |
| Toggle Switches | 2 (TSW1/TSW2) | bool |
| Slider Potentiometers | 2 (Pot1/Pot2) | 0 … 100 |
| Battery Monitor (ADS1115) | 1 | 0 … 100 % |

---

## 📡 ReceiverModule.h — Drop-in Library

Copy one file into your sketch folder. That's it.

```cpp
#include "ReceiverModule.h"

void setup() {
  receiver.begin();                         // accept any TX (broadcast)
  // receiver.begin("AA:BB:CC:DD:EE:FF");  // or lock to one TX MAC
}

void loop() {
  receiver.update();                 // handles failsafe + data-zero
  if (receiver.available()) {
    int  speed = receiver.data.Ly;   // left stick Y  → motor speed
    bool arm   = receiver.data.TSW1; // toggle 1      → arm enable
  }
}
```

**Features:** MAC filtering · auto ACK · 1 s failsafe zero · Core 2.x / 3.x compatible

---

## 🤖 Projects Using URC

<div align="center">

| Project | Notes |
|---------|-------|
| **ESP32 Hexapod** | 2× PCA9685 · 18× MG servo · tripod gait · IK |
| **Otto Biped (SiBot)** | Safe angle limits · 2S LiPo · buck regulated |
| **1D LED Racing** | 4-player · ESP-NOW buttons · NeoPixel strip |
| **1D LED Pong** | Physics engine · TM1637 score display · forfeit system |

</div>

---

## 📂 Version History

| Version | Key Addition | Status |
|---------|-------------|:------:|
| v0.4 – v0.9 | Basic joystick + ESP-NOW link | Archive |
| v1.0 | Toggle switches · pots · battery ADC | Archive |
| v1.1 | ACK watchdog · LED indicators | Archive |
| v1.2 | ST7920 SPI LCD via U8g2 | Archive |
| v1.5 | ReceiverModule.h singleton class | Archive |
| v1.9 | 4-screen display on ESP32 · sound engine | Archive |
| **[v2.0](./v2.0/README.md)** | **Nano display offload · 18-byte UART · 4 screens · sweep audio** | ✅ Current |

---

## 🗂️ Repo Structure

```
URC/
├── README.md                 ← You are here
├── ReceiverModule.h          ← Universal drop-in receiver library
├── assets/                   ← 🖼️ Add your photos here
├── v2.0/
│   ├── README.md             ← v2.0 full documentation
│   ├── ESP32_TX_v2.0/        ← Transmitter firmware
│   └── Nano_Display_v2.0/   ← KS0108 display firmware
└── versions/
    └── v0.4 … v1.9/         ← Archived versions
```

---

## 📋 Dependencies

| Side | Board | Library |
|------|-------|---------|
| Transmitter | ESP32 | Adafruit ADS1X15 |
| Display | Arduino Nano | U8g2 (`U8G2_KS0108_128X64_F`) |
| Receiver | Any ESP32 | None — just copy `ReceiverModule.h` |

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
