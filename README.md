# mecanum_lulu

A smart Mecanum-wheeled robot platform for experimentation and AI integration.

## Project Overview

`mecanum_lulu` is a personal robotics project aiming to build a triangular Mecanum-wheeled robot platform. The project is divided into multiple stages:

1. **Basic Motor Control**: Initial motor driver testing, encoder reading, and motion control with PID implementation.
2. **Sensor Integration**: Add ultrasonic sensors, IMU, and camera for environmental awareness.
3. **AI Integration**: Develop computer vision and RL-based navigation algorithms.
4. **Communication**: Establish robust communication between Raspberry Pi 5 and NodeMCU-32S.
5. **Remote Control**: Integrate remote control via Wi-Fi and web-based interface.

---

## Repository Structure

```
/ (root)
│
├── README.md               # Project introduction and documentation
├── LICENSE                 # MIT License file
├── docs/                   # Design documents, specifications, diagrams
│   ├── architecture.md     # System architecture diagrams and design decisions
│   └── TODO.txt            # Future developments and feature roadmap
│
├── hardware/               # Hardware designs and schematics
│   ├── README.md           # Hardware documentation and assembly instructions
│   ├── MCU/                # ESP32/NodeMCU circuit designs and connections
│   ├── raspberry_pi/       # Raspberry Pi 5 interfacing and GPIO configurations
│   ├── circuit_diagram/    # Complete system wiring diagrams
│   └── motor/              # Motor driver circuits and encoder wiring
│
├── src/                    # Source code for all components
│   ├── developing/         # Experimental and testing code
│   │   ├── firmware/       # Low-level firmware for motor control and sensors
│   │   └── software/       # High-level software for AI and navigation
│   ├── esp32/              # ESP32/NodeMCU code for motor control and sensor reading
│   └── raspberry/          # Raspberry Pi code for AI processing and communication
│
└── data/                   # Experiment logs, sensor readings, AI training data
                           # (e.g., motor_speed_log.csv, imu_calibration.json, training_datasets/)
```

---

## Current Hardware

- **Chassis**: Three-wheeled mecanum wheel design (custom acrylic frame)
- **Motors**: 3x DC motors with encoders (JGB37-520 12V geared motors)
- **MCU**: NodeMCU-32S (ESP32) for real-time motor control and sensor interfacing
- **Computer**: Raspberry Pi 5 (for AI processing, computer vision, and high-level control)
- **Sensors**: HC-SR04 ultrasonic sensors for obstacle detection
- **Power**: Dual power supply - 5V for MCU/Pi, 12V for motors
- **Communication**: Wi-Fi between Pi and ESP32, serial communication for debugging

---

## Development Phases

### Phase 1 - Basic Control ✅
- **Motor PID Control**: Implemented closed-loop speed control for precise movement
- **Encoder Integration**: Real-time position and velocity feedback
- **Mecanum Kinematics**: Translation and rotation control algorithms
- **Safety Features**: Motor stall detection and emergency stop functionality

### Phase 2 - Sensor Integration 🔄
- **Ultrasonic Sensors**: HC-SR04 for 360-degree obstacle detection
- **Odometry System**: Dead reckoning using encoder data and IMU fusion
- **Web Camera**: Real-time video streaming for remote monitoring
- **IMU Integration**: Gyroscope and accelerometer for orientation tracking

### Phase 3 - AI Integration 📋
- **YOLO Implementation**: Object detection and recognition
- **Path Planning**: A* or RRT algorithms for autonomous navigation
- **SLAM**: Simultaneous Localization and Mapping
- **Reinforcement Learning**: Adaptive navigation behavior

### Phase 4 - Remote Control 📋
- **Web Interface**: Browser-based control panel with live video feed
- **Mobile App**: iOS/Android app for remote operation
- **Telemetry**: Real-time sensor data visualization
- **Mission Planning**: Waypoint navigation and automated tasks

---

## Setup Instructions

### Hardware Setup
1. **Power Supply**: Connect 12V for motors, 5V for electronics
2. **Motor Connections**: Wire JGB37-520 motors to motor driver shields
3. **Encoder Wiring**: Connect encoder outputs to ESP32 interrupt pins
4. **Sensor Mounting**: Install HC-SR04 sensors at strategic positions
5. **Communication**: Establish Wi-Fi connection between Pi and ESP32

### Software Setup
1. **ESP32 Environment**: Install Arduino IDE with ESP32 board support
2. **Raspberry Pi**: Set up Raspberry Pi OS with Python 3.9+
3. **Dependencies**: Install required libraries (OpenCV, TensorFlow, etc.)
4. **Calibration**: Run motor calibration and sensor alignment procedures
5. **Testing**: Verify all systems with provided test scripts

### Quick Start
```bash
# Clone repository
git clone https://github.com/[username]/mecanum_lulu.git
cd mecanum_lulu

# Upload ESP32 firmware
# (Instructions for Arduino IDE upload)

# Run Raspberry Pi software
cd src/raspberry
python3 main.py
```

---

## License

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

## Technical Notes

- **Wiring**: Use single-core wires for minimal electromagnetic interference
- **Control Strategy**: Implement time-based sectional control for smooth operation
- **Stability**: Maintain large chassis footprint to minimize tipping during aggressive maneuvers
- **Safety**: Always implement emergency stop functionality in all control modes
- **Debugging**: Use serial communication for real-time debugging and telemetry

---

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for:
- Bug fixes and improvements
- New sensor integrations
- AI algorithm implementations
- Documentation updates

---

## Contact

**Developed by: LU**

For questions, collaboration, or technical discussions, please open an issue on GitHub or contact through the repository.

---

## Project Status

🟢 **Active Development** - Currently in Phase 2 (Sensor Integration)

Last Updated: July 2025