# System Architecture - mecanum_lulu

## Overview

The mecanum_lulu robot implements a distributed control architecture with multiple layers of hardware and software integration. The system is designed for modularity, scalability, and real-time performance.

## Hardware Architecture

### Power Distribution System
```
[Primary Power Supply]
|
├── 12V Battery Pack (Li-Po/Li-Ion)
│   ├── Motor Controllers (3x JGB37-520 12V motors)
│   ├── Servo Motors (MG996R for auxiliary systems)
│   └── Power Regulation Circuit
│
└── 5V Regulated Supply
    ├── ESP32/NodeMCU-32S (main controller)
    ├── Raspberry Pi 5 (AI processing unit)
    ├── Sensors (HC-SR04, IMU, Camera)
    └── Communication Modules
```

### Mechanical Platform
```
[Triangular Chassis Design]
|
├── Custom Acrylic Frame
│   ├── Motor Mounting Points (120° spacing)
│   ├── Electronics Compartment
│   ├── Sensor Array Mounting
│   └── Battery Compartment
│
├── Drive System
│   ├── 3x JGB37-520 DC Motors with Encoders
│   ├── Mecanum Wheels (omnidirectional movement)
│   ├── Motor Driver Shields (L298N or similar)
│   └── Encoder Interface Circuits
│
└── Auxiliary Systems
    ├── MG996R Servo (optional sail/gripper control)
    ├── LED Status Indicators
    └── Emergency Stop Button
```

### Control System Architecture
```
[Distributed Control System]
|
├── ESP32/NodeMCU-32S (Real-time Control Layer)
│   ├── Motor PWM Generation (3 channels)
│   ├── Encoder Signal Processing (interrupt-based)
│   ├── PID Control Loop Implementation
│   ├── Sensor Data Acquisition
│   │   ├── HC-SR04 Ultrasonic Sensors
│   │   ├── IMU (Accelerometer/Gyroscope)
│   │   └── Current/Voltage Monitoring
│   ├── Safety Systems
│   │   ├── Motor Stall Detection
│   │   ├── Emergency Stop Handler
│   │   └── Watchdog Timer
│   └── Communication Interface
│       ├── Wi-Fi (ESP32 AP/Station mode)
│       ├── Serial Communication (debugging)
│       └── I2C/SPI (sensor communication)
│
└── Raspberry Pi 5 (High-level Processing Layer)
    ├── AI/ML Processing
    │   ├── Computer Vision (OpenCV, YOLO)
    │   ├── Path Planning Algorithms
    │   ├── SLAM Implementation
    │   └── Reinforcement Learning
    ├── System Coordination
    │   ├── Mission Planning
    │   ├── State Management
    │   └── Data Logging
    ├── User Interface
    │   ├── Web-based Control Panel
    │   ├── Mobile App Backend
    │   └── Telemetry Dashboard
    └── External Communication
        ├── Wi-Fi/Ethernet Connectivity
        ├── Cloud Data Sync
        └── Remote Monitoring
```

## Software Architecture

### ESP32 Firmware Structure
```
[Real-time Control Loop - 100Hz]
├── Sensor Reading Task
├── PID Control Task
├── Motor Control Task
├── Communication Task
└── Safety Monitor Task

[Interrupt Service Routines]
├── Encoder Pulse Counting
├── Emergency Stop Handler
└── Watchdog Timer Reset
```

### Raspberry Pi Software Stack
```
[Application Layer]
├── Mission Control System
├── Web Interface (Flask/Django)
├── AI Processing Pipeline
└── Data Management System

[Middleware Layer]
├── Communication Manager (ESP32 ↔ Pi)
├── Sensor Data Fusion
├── State Machine Controller
└── Safety Monitor

[Hardware Abstraction Layer]
├── Camera Interface
├── Network Management
└── File System Management
```

## Communication Architecture

### Inter-System Communication
```
[ESP32] ←→ [Raspberry Pi 5]
    ↓           ↓
Wi-Fi Network Protocol
    ↓           ↓
JSON/Binary Message Format
    ↓           ↓
Command/Status Exchange
```

### Communication Protocols
- **ESP32 ↔ Raspberry Pi**: Wi-Fi (UDP/TCP) for low-latency command transfer
- **User ↔ System**: HTTP/WebSocket for web interface
- **Debugging**: Serial UART for development and troubleshooting
- **Sensors**: I2C/SPI for high-speed sensor data acquisition

### Message Structure
```json
{
  "timestamp": "2025-07-18T10:30:00Z",
  "message_type": "motor_command",
  "payload": {
    "motor_speeds": [100, -50, 75],
    "servo_angle": 90,
    "safety_enabled": true
  },
  "sequence_id": 12345
}
```

## Control Flow

### Phase 1: Basic Control (Current Implementation)
```
1. Initialize ESP32 and calibrate sensors
2. Establish communication with Raspberry Pi
3. Enter main control loop:
   a. Read encoder feedback
   b. Receive velocity commands
   c. Execute PID control
   d. Apply motor PWM signals
   e. Monitor safety conditions
   f. Send status updates
```

### Phase 2: Sensor Integration (In Progress)
```
1. Multi-sensor data fusion (IMU + Ultrasonic)
2. Obstacle detection and avoidance
3. Odometry calculation and dead reckoning
4. Environmental mapping
5. Real-time camera feed processing
```

### Phase 3: AI Integration (Planned)
```
1. YOLO object detection pipeline
2. Path planning with obstacle avoidance
3. SLAM for environment mapping
4. Reinforcement learning for adaptive behavior
5. Multi-objective optimization
```

### Phase 4: Remote Control (Planned)
```
1. Web-based control interface
2. Mobile app development
3. Telemetry and monitoring system
4. Mission planning interface
5. Cloud connectivity and data sync
```

## Safety Architecture

### Hardware Safety Features
- **Emergency Stop**: Physical button for immediate motor shutdown
- **Current Monitoring**: Overcurrent protection for motor circuits
- **Voltage Monitoring**: Low battery detection and warnings
- **Thermal Protection**: Temperature monitoring for motor drivers

### Software Safety Features
- **Watchdog Timer**: Automatic reset on system hang
- **Motion Limits**: Maximum speed and acceleration constraints
- **Collision Detection**: Emergency stop on obstacle detection
- **Communication Timeout**: Safe mode on communication loss
- **State Validation**: Input sanity checks and error handling

## Development Roadmap

### Immediate Priorities (Phase 1 Complete)
- ✅ Basic motor control with PID
- ✅ Encoder feedback integration
- ✅ Serial communication interface
- ✅ Safety system implementation

### Current Development (Phase 2)
- 🔄 Sensor integration and calibration
- 🔄 Wi-Fi communication between ESP32 and Pi
- 🔄 Basic obstacle detection
- 🔄 Odometry and localization

### Future Development (Phase 3-4)
- 📋 Computer vision implementation
- 📋 AI-based navigation
- 📋 Web/mobile interface
- 📋 Advanced mapping and planning

## Performance Specifications

### Real-time Requirements
- **Control Loop**: 100 Hz (10ms cycle time)
- **Sensor Reading**: 50 Hz (20ms cycle time)
- **Communication**: 20 Hz (50ms cycle time)
- **Vision Processing**: 10 Hz (100ms cycle time)

### Accuracy Requirements
- **Position Accuracy**: ±2cm absolute, ±1cm relative
- **Orientation Accuracy**: ±2 degrees
- **Speed Control**: ±5% of target speed
- **Response Time**: <100ms for emergency stop

## Troubleshooting and Debugging

### Common Issues and Solutions
1. **Motor Stall Detection**: Check encoder feedback and current monitoring
2. **Communication Loss**: Verify Wi-Fi connection and message formatting
3. **Sensor Calibration**: Use provided calibration procedures
4. **Power Issues**: Monitor voltage levels and current draw
5. **Navigation Errors**: Check odometry calculations and sensor alignment

### Debug Tools
- **Serial Monitor**: Real-time system status and error messages
- **Web Dashboard**: Live telemetry and system health monitoring
- **Log Files**: Detailed operation history and error tracking
- **Test Scripts**: Automated system verification and validation

---

**Last Updated**: July 2025  
**Version**: 2.0  
**Status**: Phase 2 Development