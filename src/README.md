# 🤖 Omnidirectional Robot Control System

A complete robotics system for controlling an omnidirectional robot with AI object tracking, web interface, and real-time video streaming.

## 📂 Project Structure

```
.
├── 📁 firmware/                    # ESP32 Motor Controller
│   └── mecanum_controller.ino      # Motor control with ultrasonic sensor
│
├── 📁 software/                    # ROS2 Robot Nodes (Raspberry Pi)
│   ├── keyboard.py                 # Direct keyboard control
│   ├── teleporter.py              # Simple UART teleop
│   ├── UART.py                    # Motor communication bridge
│   ├── usb_cam.py                 # Camera publisher node
│   └── YOLO.py                    # AI object tracking
│
└── 📁 web_controller/              # Web Control Interface
    ├── app_controller.py          # Main control server
    ├── app_video.py              # Video streaming server
    ├── server.py                 # ROS2 server (runs on Pi)
    └── socket_client.py          # Socket communication client
```

## 🎯 What Each Component Does

### 📱 Web Controller
The web interface that lets you control the robot from any browser:

- **`app_controller.py`**: Main control server (port 5000) - handles movement commands, mode switching, and robot status
- **`app_video.py`**: Video streaming server (port 5001) - streams live camera feed from the robot
- **`server.py`**: ROS2 bridge on the robot - translates web commands to ROS2 messages
- **`socket_client.py`**: Communication handler - manages persistent connection between web and robot

### 🧠 Software (ROS2 Nodes)
The brain of the robot running on Raspberry Pi:

- **`YOLO.py`**: AI vision system - detects and tracks objects using deep learning, sends tracking data for follow mode
- **`usb_cam.py`**: Camera driver - captures video from USB camera and publishes to ROS2 network
- **`UART.py`**: Motor bridge - converts ROS2 velocity commands to motor control signals, handles emergency stops
- **`keyboard.py`**: Direct control - allows controlling robot with keyboard when connected directly to Pi
- **`teleporter.py`**: Simple UART sender - basic script for testing motor communication

### ⚙️ Firmware
The motor controller running on ESP32:

- **`mecanum_controller.ino`**: 
  - Controls 3 omnidirectional wheels with PID speed control
  - Reads encoder feedback for precise movement
  - Monitors ultrasonic sensor for obstacle detection
  - Emergency stops when objects detected within 20cm
  - Sends distance data back to Pi every 100ms

## 🔄 How It All Works Together

```
1. You open the web interface on your laptop
                    ↓
2. Send movement commands or switch to AI follow mode
                    ↓
3. Commands travel through socket to ROS2 server on Pi
                    ↓
4. ROS2 publishes velocity commands to motor bridge
                    ↓
5. Motor bridge sends commands via UART to ESP32
                    ↓
6. ESP32 controls motors with PID feedback loop
                    ↓
7. Ultrasonic sensor monitors for obstacles
                    ↓
8. Camera streams video back to web interface
                    ↓
9. YOLO processes video for object tracking (if in follow mode)
```

## 🎮 Features

### Control Modes
- **Manual Mode**: Direct control using web interface or keyboard
- **Follow Mode**: AI automatically tracks and follows detected objects
- **Emergency Stop**: Automatic stop when obstacles detected <20cm

### Safety Systems
- Ultrasonic distance monitoring
- Automatic emergency stop
- 2-second timeout recovery
- Gradual speed ramp-up after stops

### Real-time Feedback
- Live video streaming at 30 FPS
- Distance measurements every 100ms
- Motor speed monitoring
- Connection status updates

## 📡 Communication Flow

```
Web Browser ←─HTTP─→ Flask Servers ←─Socket─→ ROS2 Server
                            ↓
                     ROS2 Topics Network
                    /cmd_vel  |  /image_raw
                        ↓     |     ↓
                    UART.py   |  YOLO.py
                        ↓     |     ↓
                    ESP32     | Object Tracking
                        ↓
                    Motors + Sensors
```

## 🔧 Key Technologies

- **ROS2**: Robot Operating System for node communication
- **Flask**: Web framework for control interface
- **YOLO**: Deep learning model for object detection
- **WebSocket**: Real-time bidirectional communication
- **ESP32**: Microcontroller for motor control
- **PID Control**: Precise motor speed regulation

---

This system enables remote robot control through a web browser with AI-powered object following and comprehensive safety features.