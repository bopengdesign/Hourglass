# IoT-Based Smart Hourglass System

This repository contains the embedded firmware and 3D model developed for the bachelor's thesis: **"Design and Implementation of an IoT-Based Smart Hourglass"** at Häme University of Applied Sciences (HAMK).

## 📌 Project Overview
The smart hourglass is an electro-mechanical timing and sand-level monitoring prototype. It regulates granular flow using an adjustable multi-orifice disk and tracks sand accumulation height in real time through calibrated sensor measurements and embedded control.

### Key Features
* **Adjustable Flow Control:** Multi-orifice disk mechanism allowing discrete flow-rate selection.
* **Feedback:** Rotary encoder input with quadrature edge detection, MOSFET-controlled LED strip and PWM-driven passive buzzer for alerts.

---

## 📂 Repository Structure

```
├── SourceCode/              # Arduino / C++ embedded source code (.ino)
├── Diagrams/                # Schematic and wiring diagrams
├── 3D Model/                # Standard solid models for CNC/modification (.step)
└── README.md                # Project documentation
```

---

## 🛠️ Hardware & Software Requirements
* **Microcontroller:** Arduino Uno ESP32
* **Sensors & Actuators:** VL53L0X * 2, Rotary Encoder, Passive Buzzer, LED Strip, MOSFET
* **IDE & Libraries:** Arduino IDE (v2.3.8) — Required libraries: Arduino.h, Wire.h, Adafruit_VL53L0X.h, AdafruitIO_WiFi.h

---

## 🚀 Quick Start
1. **3D Printing / Machining:** Use `.step` files for high-precision CNC machining / laser cutting of the multi-orifice disk.
2. **Firmware Upload:** Open the `.ino` sketch inside the `SourceCode/` folder using Arduino IDE, select your board and port, and click **Upload**.
3. **Initial Calibration:** Upon startup, make sure the encoder phase arrow points at 5 min, then press for 2 seconds to confirm phase 0.

---

## 📄 Citation / Author
* **Author:** Bo Peng
* **Year:** 2026
* **License:** MIT License (Open for academic and educational use)
