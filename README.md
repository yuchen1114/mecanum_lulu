# mecanum\_lulu

A smart Mecanum-wheeled robot platform for experimentation and AI integration.

## Project Overview

`mecanum_lulu` is a personal robotics project aiming to build a triangular Mecanum-wheeled robot platform. The project is divided into multiple stages:

1. **Basic Motor Control**: Initial motor driver testing, encoder reading, and motion control.
2. **Sensor Integration**: Add ultrasonic sensors, IMU, and camera (future stage).
3. **AI Integration**: Develop computer vision and RL-based navigation.
4. **Remote Control & Communication**: Integrate remote control via Wi-Fi and app.

---

## Repository Structure

```
/ (root)
│
├── README.md               # Project introduction
├── LICENSE                 # MIT License file
├── docs/                   # Design documents, specs, diagrams
│   └── architecture.md     # System architecture diagrams
│
├── hardware/               # Hardware designs
│   ├── schematics/         # Electrical schematics
│   ├── pcb/                # PCB files (if any)
│   └── chassis-design/     # CAD files for chassis
│
├── firmware/               # ESP32 / microcontroller code
│   └── src/                # Source code for motor, sensor control
│
├── software/               # Higher-level control (AI, remote control)
│   └── vision/             # Computer vision models (future)
│
├── tests/                  # Test scripts for hardware/software modules
│                           # (e.g. motor_test.ino, encoder_read.py)
│
└── data/                   # Experiment logs, sensor readings, AI training data
│                           # (e.g. motor_speed_log.csv, imu_calibration.json)
```

---

## Current Hardware

- Chassis: Triangular mecanum wheel design (custom acrylic)
- Motors: 3 DC motors with encoders
- Servo: MG996R (for potential sail or manipulator)
- Controller: ESP32 (motor + sensor control)
- Computer: Raspberry Pi (for future AI tasks)
- Power: 3.7V (controller), 7.4V (motors/servo)

---

## Development Phases

### Phase 1 - Basic Control

-

### Phase 2 - Sensor Integration

-

### Phase 3 - AI Integration

-

### Phase 4 - Remote Control

-

---

## Setup Instructions

> *To be filled as development progresses*

---

## License

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

## Notes

- Use single-core wires for minimal interference.
- Use time-based control (sectional control) for sail control (optional subsystem).
- Keep the chassis large to minimize tipping during movement.

---

## Contact

Developed by: **LU**

For questions or collaboration, open an issue on GitHub.
