# 🤖 ROS2 Robot Remote Controller

A comprehensive web-based control system for ROS2 robots with real-time video streaming, multiple control modes, and remote operation capabilities.

## 📋 Table of Contents
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Control Modes](#control-modes)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## ✨ Features

### 🎮 Control System
- **Three Control Modes**: Manual, Gyro (tilt), and Object Following
- **Real-time Response**: Low-latency command transmission
- **Hold-to-Move**: Continuous movement with safety timeout
- **Keyboard Support**: WASD and arrow key controls
- **Mobile-Friendly**: Touch controls and gyroscope support

### 📹 Video Streaming
- **Live Camera Feed**: Real-time video from robot camera
- **Separate Video Server**: Dedicated streaming on port 5001
- **Performance Monitoring**: FPS counter and connection status
- **Snapshot Capture**: Save camera frames locally
- **Fullscreen Mode**: Immersive viewing experience

### 🔧 System Features
- **Modular Architecture**: Separate servers for video and control
- **Auto-Reconnection**: Handles network interruptions gracefully
- **Web-Based Interface**: No app installation required
- **Cross-Platform**: Works on desktop, tablet, and mobile
- **ROS2 Integration**: Native ROS2 node communication

## 🏗️ System Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│                 │         │                  │         │                 │
│  Web Browser    │◄────────┤  PC/Server       ├────────►│  Raspberry Pi   │
│  (Client)       │  HTTP   │                  │  Socket │  (Robot)        │
│                 │         │                  │         │                 │
├─────────────────┤         ├──────────────────┤         ├─────────────────┤
│ • controller.html        │ • app_controller.py        │ • server.py      │
│ • video.html             │ • app_video.py            │ • ROS2 Node      │
│ • JavaScript             │ • socket_client.py        │ • Camera         │
│                 │         │                  │         │ • Motors         │
└─────────────────┘         └──────────────────┘         └─────────────────┘
     Port 5000/5001              Port 5000/5001              Port 8888/8889
```

## 📦 Prerequisites

### On Raspberry Pi (Robot)
- Ubuntu 22.04 or compatible Linux distribution
- ROS2 Humble or later
- Python 3.8+
- OpenCV (`cv2`)
- Required ROS2 packages:
  ```bash
  sudo apt install ros-humble-cv-bridge ros-humble-image-transport
  ```

### On PC (Controller)
- Python 3.8+
- Flask web framework
- OpenCV for Python
- NumPy

## 🚀 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/ros2-robot-controller.git
cd ros2-robot-controller
```

### 2. Install Python Dependencies

#### On PC (Controller Side)
```bash
pip install flask opencv-python numpy
```

#### On Raspberry Pi (Robot Side)
```bash
pip install opencv-python numpy
# ROS2 dependencies should be installed via apt (see Prerequisites)
```

### 3. Set Up File Structure
```
ros2-robot-controller/
├── README.md
├── config.py              # Configuration file
├── app_controller.py      # Robot control server
├── app_video.py          # Video streaming server
├── socket_client.py      # Socket communication client
├── server.py            # Pi-side ROS2 server
├── templates/
│   ├── controller.html  # Control interface
│   └── video.html      # Video interface
└── static/             # (Optional) Static assets
```

## ⚙️ Configuration

### 1. Update Network Settings

Edit `config.py` to match your network configuration:

```python
# Robot Controller Configuration
PI_IP = "192.168.16.154"  # Your Pi's IP address
PI_PORT = 8888            # Command port
VIDEO_PORT = 8889         # Video streaming port

# Web Server Settings
WEB_HOST = "0.0.0.0"      # Listen on all interfaces
WEB_PORT = 5000           # Controller web port
VIDEO_WEB_PORT = 5001     # Video web port
```

### 2. Configure ROS2 Topics (server.py)

Ensure the ROS2 topics match your robot setup:

```python
# Publishers
self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
self.mode_publisher = self.create_publisher(String, '/robot_mode', 10)

# Subscribers
self.image_subscriber = self.create_subscription(
    Image, '/image_raw', self.image_callback, image_qos_profile
)
```

## 🎯 Usage

### Starting the System

#### 1. On Raspberry Pi (Robot)

First, source your ROS2 environment:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # If using a workspace
```

Start the ROS2 server:
```bash
python3 server.py
```

#### 2. On PC (Controller)

Start both servers in separate terminals:

**Terminal 1 - Video Server:**
```bash
python app_video.py
```

**Terminal 2 - Control Server:**
```bash
python app_controller.py
```

### Accessing the Interfaces

- **Robot Controller**: http://localhost:5000
- **Video Stream**: http://localhost:5001

To access from other devices on the network, replace `localhost` with your PC's IP address.

## 🎮 Control Modes

### 🕹️ Manual Mode
- **Arrow Keys / WASD**: Move robot
- **Space**: Emergency stop
- **Mouse/Touch**: Click and hold directional buttons
- **Hold-to-Move**: Continuous movement while button is pressed

### 📱 Gyro Mode
- **Tilt Forward**: Move forward
- **Tilt Backward**: Move backward
- **Tilt Left**: Turn left
- **Tilt Right**: Turn right
- **Level Position**: Stop
- Requires device with gyroscope (smartphone/tablet)

### 👁️ Follow Mode
- Automatically tracks and follows detected objects
- Displays tracking status and error from center
- Robot moves forward when object is centered
- Turns to keep object in view

## 📡 API Reference

### Control Endpoints

#### `POST /set_mode`
Change robot control mode.
```json
Request:
{
  "mode": "manual" | "gyro" | "follow"
}

Response:
{
  "status": "success",
  "mode": "manual",
  "message": "Mode switched to manual"
}
```

#### `POST /move`
Send movement command (manual mode).
```json
Request:
{
  "direction": "forward" | "backward" | "left" | "right" | "stop"
}
```

#### `POST /gyro_control`
Send gyro control data.
```json
Request:
{
  "pitch": -45.5,
  "roll": 12.3,
  "command": "forward"
}
```

#### `GET /status`
Get system status.
```json
Response:
{
  "mode": "manual",
  "pi_status": {
    "connected": true,
    "status": "online"
  },
  "pi_ip": "192.168.16.154"
}
```

### Socket Commands (Pi ↔ PC)

| Command | Description | Response |
|---------|-------------|----------|
| `MODE:manual` | Set manual mode | `MODE_OK:manual` |
| `MOVE:forward` | Move forward | `MOVE_OK:forward` |
| `GYRO:pitch=X,roll=Y` | Gyro data | `GYRO_OK` |
| `STATUS` | Get status | `STATUS_OK:{json}` |
| `PING` | Check connection | `PONG` |

## 🔧 Troubleshooting

### Connection Issues

**Problem**: Cannot connect to Raspberry Pi
- Check Pi IP address is correct in `config.py`
- Ensure Pi and PC are on same network
- Verify firewall settings allow ports 8888-8889
- Test with ping: `ping 192.168.16.154`

**Problem**: Video stream not working
- Check camera is connected: `ls /dev/video*`
- Verify ROS2 camera node is publishing to `/image_raw`
- Test with: `ros2 topic echo /image_raw`

### Performance Issues

**Problem**: High latency in controls
- Reduce video quality in server.py:
  ```python
  encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]  # Lower quality
  ```
- Check network bandwidth
- Consider using wired connection

**Problem**: Robot stops unexpectedly
- Increase timeout in server.py:
  ```python
  COMMAND_TIMEOUT = 1.0  # Increase from 0.5
  ```

### ROS2 Issues

**Problem**: Topics not found
```bash
# List all topics
ros2 topic list

# Check topic is publishing
ros2 topic hz /cmd_vel
```

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push to branch: `git push origin feature-name`
5. Submit a Pull Request

### Development Guidelines
- Follow PEP 8 for Python code
- Add comments for complex logic
- Update README for new features
- Test on both desktop and mobile

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS2 community for the excellent robotics framework
- Flask team for the lightweight web framework
- OpenCV contributors for computer vision tools

## 📞 Support

For issues and questions:
- Open an issue on GitHub
- Check existing issues for solutions
- Provide system details and error logs

---

**Made with ❤️ for the robotics community**