# 🤖 ROS2 Omnidirectional Robot System

A complete ROS2-based control system for an omnidirectional robot featuring web-based control, AI-powered object tracking, and real-time video streaming.

## 📋 System Overview

This project implements a full-stack robotics system with:
- **Web-based remote control** with live video feed
- **AI object tracking** using YOLO
- **Omnidirectional movement** with 3-wheel mecanum drive
- **Emergency stop** with ultrasonic sensors
- **Manual and autonomous modes**

## 🏗️ Architecture

```
┌──────────────────┐         ┌──────────────────┐
│   Web Browser    │◄────────►│  Controller App  │
│  (Port 5000)     │   HTTP   │  (app_controller)│
└──────────────────┘         └─────────┬──────────┘
                                       │ Socket
┌──────────────────┐         ┌─────────▼──────────┐
│   Web Browser    │◄────────►│   Video Stream   │
│  (Port 5001)     │   HTTP   │   (app_video)    │
└──────────────────┘         └─────────┬──────────┘
                                       │ Socket
                            ┌──────────▼──────────┐
                            │   Robot (Pi/ROS2)  │
                            │     (server.py)    │
                            └──────────┬──────────┘
                                       │
                    ┌──────────────────┼──────────────────┐
                    │                  │                  │
            ┌───────▼────────┐ ┌──────▼───────┐ ┌────────▼────────┐
            │  USB Camera    │ │  YOLO Node   │ │   UART Bridge   │
            │ (usb_cam.py)   │ │ (YOLO.py)    │ │  (UART.py)      │
            └────────────────┘ └──────────────┘ └────────┬────────┘
                                                          │
                                                ┌─────────▼─────────┐
                                                │  Motor Controller │
                                                │    (ESP32/MCU)    │
                                                └───────────────────┘
```

## 🚀 Features

### Control Modes
- **Manual Mode**: Direct control via web interface or keyboard
- **Follow Mode**: AI-powered object tracking with distance control
  - Maintains 30-50cm optimal following distance
  - Automatic obstacle avoidance
  - Speed adjustment based on distance

### Safety Features
- **Emergency Stop**: Ultrasonic sensor-based collision prevention
- **Command Timeout**: Automatic stop after 500ms without commands
- **Connection Monitoring**: Auto-reconnection and status updates
- **Distance-based Speed Control**: Slower speeds when approaching objects

### Video & AI
- **Live Streaming**: 640x480 @ 30 FPS camera feed
- **YOLO Object Detection**: Custom-trained model for object tracking
- **Visual Feedback**: Real-time tracking visualization

## 📁 Project Structure

```
robot-system/
├── Web Control (PC/Laptop)
│   ├── app_controller.py    # Main control server (Port 5000)
│   ├── app_video.py         # Video streaming server (Port 5001)
│   └── socket_client.py     # Socket communication client
│
├── Robot (Raspberry Pi)
│   ├── server.py            # ROS2 control server
│   ├── YOLO.py             # Object detection & tracking
│   ├── usb_cam.py          # Camera publisher node
│   ├── UART.py             # Motor controller bridge
│   ├── keyboard.py         # Direct keyboard control
│   └── teleporter.py       # Simple UART teleop
│
└── Configuration
    └── best.pt             # YOLO model weights
```

## 🛠️ Installation

### Prerequisites

#### PC/Laptop (Control Station)
```bash
# Python packages
pip install flask opencv-python numpy
```

#### Raspberry Pi (Robot)
```bash
# ROS2 Humble or newer
sudo apt update
sudo apt install ros-humble-desktop

# Python packages
pip install opencv-python ultralytics pyserial numpy

# ROS2 packages
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

### Setup Instructions

1. **Clone the repository**
```bash
git clone <repository-url>
cd robot-system
```

2. **Configure Network Settings**

Edit IP addresses in the control apps:
```python
# app_controller.py
pi_ip = "192.168.16.154"  # Your robot's IP

# app_video.py  
PI_IP = "192.168.16.154"  # Your robot's IP
```

3. **Configure Serial Port**

On the Raspberry Pi, identify your motor controller's serial port:
```bash
ls /dev/tty*
# Look for /dev/ttyUSB0 or /dev/ttyAMA0
```

Update in `UART.py`:
```python
self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Your serial port
```

4. **Place YOLO Model**

Copy your trained YOLO model to the robot:
```bash
# Update path in YOLO.py
MODEL_PATH = "/home/lulu/mecanum_lulu/log/YOLO_model/lip/best.pt"
```

## 🎮 Usage

### Starting the System

#### 1. On the Robot (Raspberry Pi)

Start ROS2 nodes in separate terminals:

```bash
# Terminal 1: Camera feed
ros2 run your_package usb_cam.py

# Terminal 2: Main server
ros2 run your_package server.py

# Terminal 3: Motor communication
ros2 run your_package UART.py

# Terminal 4: Object tracking (for follow mode)
ros2 run your_package YOLO.py
```

Or use a launch file:
```bash
ros2 launch your_package robot_system.launch.py
```

#### 2. On the Control PC

Start the web servers:

```bash
# Terminal 1: Control server
python app_controller.py

# Terminal 2: Video server
python app_video.py
```

#### 3. Access the Web Interface

Open your browser and navigate to:
- **Control Interface**: http://localhost:5000
- **Video Stream**: http://localhost:5001

### Control Instructions

#### Manual Mode
- **Arrow Keys / WASD**: Movement control
- **Space**: Emergency stop
- **Click & Hold**: Continuous movement via buttons

#### Follow Mode
1. Switch to "Object Following" mode
2. Point camera at object to track
3. Robot will automatically:
   - Center the object in view
   - Maintain safe distance
   - Stop if too close

### Direct Keyboard Control (On Robot)

For testing without web interface:
```bash
ros2 run your_package keyboard.py
```

## ⚙️ Configuration

### Robot Parameters

Edit `server.py` to adjust:
```python
# Movement speeds
self.linear_speed = 0.2   # m/s
self.angular_speed = 1.0  # rad/s

# Follow mode
self.min_follow_distance = 30.0   # cm - stop distance
self.safe_follow_distance = 50.0  # cm - optimal distance
self.follow_kp = 0.003            # Turning gain
```

### Motor Controller

Edit `UART.py` for your robot's kinematics:
```python
# Robot dimensions
self.radius_base = 110.0  # mm - distance from center to wheel

# Kinematics matrix for 3-wheel omnidirectional
self.forward_kinematics = np.array([
    [0, 1, -self.radius_base],
    [-math.sin(math.pi/3), -math.cos(math.pi/3), -self.radius_base],
    [math.cos(math.pi/6), -math.sin(math.pi/6), -self.radius_base]
])
```

### Camera Settings

Edit `usb_cam.py`:
```python
self.declare_parameter('camera_index', 0)      # Camera device
self.declare_parameter('frame_width', 1920)    # Resolution
self.declare_parameter('frame_height', 1080)
self.declare_parameter('fps', 30)              # Frame rate
```

## 🔧 Troubleshooting

### Camera Issues
```bash
# List available cameras
ls /dev/video*

# Test camera
ros2 run your_package usb_cam.py --ros-args -p camera_index:=0
```

### Serial Communication
```bash
# Check serial permissions
sudo usermod -a -G dialout $USER
# Logout and login again

# Test serial port
screen /dev/ttyUSB0 115200
```

### Network Connection
```bash
# Check robot IP
hostname -I

# Test connection from PC
ping 192.168.16.154

# Check ports
netstat -tuln | grep -E '5000|5001|8888|8889'
```

### YOLO Model
```bash
# Verify model path
ls -la /home/lulu/mecanum_lulu/log/YOLO_model/lip/best.pt

# Test YOLO node
ros2 run your_package YOLO.py
```

## 📊 Performance Metrics

- **Control Latency**: <100ms typical
- **Video Latency**: <200ms typical  
- **Object Detection**: ~30 FPS with YOLO
- **Command Rate**: 10-20 Hz
- **Emergency Stop Response**: <50ms

## 🔌 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/image_raw` | sensor_msgs/Image | Camera feed |
| `/tracker_data` | std_msgs/Float64 | Object tracking error |
| `/robot_mode` | std_msgs/String | Current control mode |
| `/robot_status` | std_msgs/String | System status JSON |
| `/emergency_stop` | std_msgs/Bool | Emergency stop status |

## 📝 Serial Protocol

### To Motor Controller
```
Format: vel1,vel2,vel3\n
Example: 100.5,-50.2,75.0\n
```

### From Motor Controller
```
VEL:vel1,vel2,vel3,timestamp    # Current velocities
DIST:distance,timestamp          # Ultrasonic distance (cm)
ESTOP:ACTIVE/INACTIVE           # Emergency stop status
```

## 🛡️ Safety Guidelines

1. **Always test in safe environment first**
2. **Keep emergency stop accessible**
3. **Monitor ultrasonic sensor readings**
4. **Set appropriate speed limits**
5. **Ensure stable network connection**
6. **Regular system health checks**

## 🤝 Contributing

Feel free to submit issues and enhancement requests!

## 📄 License

MIT License - See LICENSE file for details

## 🙏 Acknowledgments

- ROS2 Community
- Ultralytics YOLO
- Flask Framework
- OpenCV

---

**For questions or support, please open an issue on GitHub.**