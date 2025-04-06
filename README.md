# Distributed-Real-Time-Control-Systems

This repository contains the implementation of a **Distributed Real-Time Control System** for a **Smart Lighting** network, developed as part of the *Distributed Real-Time Control Systems* course at **Instituto Superior Técnico**, during the 2024/2025 Spring Semester.

## 📌 Project Overview

The system simulates a network of smart luminaires controlling desk illumination in an office-like environment. Each luminaire node:

- Uses an LED to illuminate the desk.
- Measures reflected light using an LDR sensor.
- Executes local real-time control using a PI controller.
- Communicates with other nodes via CAN-BUS.
- Evaluates performance through energy, flicker, and visibility metrics.

The main objective is to maintain visual comfort while minimizing energy consumption, under dynamic environmental and occupancy conditions.

## 🧠 Features

- 💡 **PI Controller with Feedforward & Anti-Windup:** Maintains desired illuminance levels with fast, smooth response.
- 🧪 **Sensor Calibration:** Converts LDR voltage to LUX using a log-log relationship.
- 📉 **Real-Time Metrics:** Monitors energy usage, flicker, and visibility error.
- 🔀 **Multicore Execution:** Separates control and communication across Raspberry Pi Pico cores.
- 🔌 **CAN-BUS Support:** Enables inter-node message exchange for distributed control.
- 🧾 **User Interface:** Serial command-line interface to interact with nodes.

## 🛠️ System Architecture

Each luminaire node is built on a Raspberry Pi Pico and includes:
- An **LED driver** (PWM controlled).
- An **LDR-based luxmeter**.
- A **CAN-BUS transceiver** (MCP2515).
- Local control logic and real-time metrics computation.

## 📦 Repository Contents

```
├── Node_0/                  # Microcontroller C++ code
├── Node_1/                 # Diagrams, calibration graphs, performance plots
├── README.md             # Project overview (this file)
└── report.pdf            # Full technical report (Stage 1)
```

## 🧑‍🎓 Authors

**José Pedro da Cunha Rodrigues**
**Miguel Rodrigues Ferreira**
**Jaime Carriço Almeida de Sousa Alvarez**
Email: [jpcr400@gmail.com](mailto:jpcr400@gmail.com)

## 🏫 Course

*Distributed Real-Time Control Systems*  
Master's in Electrical and Computer Engineering  
Instituto Superior Técnico, ULisboa  
Spring Semester, 2024/2025


