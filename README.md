# Three-Factor Smart Lock System (ESP32)

A multi-factor smart lock built on an ESP32 using joystick navigation, rotary encoder combinations, and rhythm-based knock authentication. The system uses Dynamic Time Warping (DTW) to perform tempo-invariant knock pattern matching and is designed around a deterministic finite state machine.

---

## Features

- Three-factor authentication using independent input modalities
- Tempo-invariant knock pattern recognition using Dynamic Time Warping (DTW)
- Finite state machine with timeouts and lockout protection
- Interactive on-device calibration menu
- Real-time user feedback via 16x2 I2C LCD
- Robust input handling with multi-stage debouncing
- Polling-based rotary encoder decoding for reliability

---

## Authentication Flow

IDLE
→ JOYSTICK_AUTH
→ ENCODER_AUTH
→ KNOCK_AUTH
→ UNLOCKED

(on failure)
→ LOCKED_OUT



- Each stage has a 30-second timeout
- 3 failed attempts trigger a 60-second lockout
- Lockout countdown is displayed on the LCD

---

## Authentication Methods

### 1. Joystick Path Authentication
- Analog joystick mapped to a normalized range (-10 to +10)
- Deadzones and hysteresis to suppress noise
- Direction debouncing (150 ms)
- Minimum sequence length enforced
- LCD displays progress (e.g. `2/3 UP`)

### 2. Rotary Encoder Combination
- Safe-dial style combination using a quadrature encoder
- Polling-based decoding (10 ms loop)
- Position reset after each entry
- ±3 tick tolerance to accommodate human error
- Button-confirmed input steps

### 3. Knock Pattern Authentication
- Rhythm-based authentication using a knock sensor
- Dynamic Time Warping (DTW) for pattern matching
- Tempo-independent recognition
- Robust against inconsistent human timing

**DTW Performance**
- Time complexity: O(n × m)
- Execution time: < 500 µs on ESP32
- Typical pattern length: 3–6 knocks

---

## Hardware

| Component | Description |
|--------|------------|
| ESP32 | Main microcontroller |
| KY-023 | Analog joystick |
| KY-040 | Rotary encoder |
| KY-031 | Knock sensor |
| 16x2 LCD (I2C) | User feedback |

### Pin Configuration

Joystick:
VRx → GPIO 34
VRy → GPIO 35
SW → GPIO 32

Encoder:
CLK → GPIO 25
DT → GPIO 26
SW → GPIO 27

Knock Sensor:
DO → GPIO 33

LCD:
SDA → GPIO 21
SCL → GPIO 22


---

## Software Architecture

### Finite State Machine
- Explicit states prevent authentication bypass
- State transitions only occur on validated input
- Failure handling routes system to lockout state

### Input Handling
- Multi-stage debouncing for mechanical sensors
- Edge-detected button presses
- Polling-based encoder decoding for direction reliability

---

## Calibration

Credentials can be reprogrammed directly on-device without reflashing firmware.

### Enter Calibration Mode
- Hold encoder button for 3 seconds

### Calibration Options
- Joystick: record directional sequence
- Encoder: record position-based combination
- Knock: record rhythm intervals

All credentials are validated for minimum complexity before saving.

---

## Performance

| Metric | Value |
|------|------|
| Authentication time | < 5 s (typical) |
| DTW execution | < 500 µs |
| Main loop period | ~10 ms |
| RAM usage | < 10 KB |
| Flash usage | ~1 MB |

---

## Security Features

- Multi-factor authentication
- Global failed-attempt tracking
- Lockout after repeated failures
- Tolerance-aware matching to resist replay attacks
- Clear state isolation between authentication stages

---

## Future Improvements

- Non-volatile credential storage (EEPROM / NVS)
- Solenoid or servo lock control
- Tamper detection
- Remote logging over WiFi
- Encrypted credential storage

---

## Summary

This project demonstrates:
- Embedded C++ system design
- Real-time sensor processing
- Signal processing with DTW
- Robust human-input handling
- Security-focused architecture
