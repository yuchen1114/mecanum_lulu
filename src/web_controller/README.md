# 🤖 Robot Web Controller

A modern, responsive web-based control system for ROS2-enabled robots with real-time video streaming, manual control, and AI-powered object following capabilities.

## ✨ Features

### 🎮 Control Modes
- **Manual Mode**: Direct control using on-screen buttons or keyboard (WASD/Arrow keys)
- **Follow Mode**: AI-powered object tracking with distance-based speed control
  - Maintains safe following distance (30-50cm optimal range)
  - Automatic obstacle avoidance with ultrasonic sensors
  - Real-time distance visualization

### 📹 Live Video Streaming
- Real-time camera feed from robot
- 640x480 resolution at ~30 FPS
- Separate video server for optimal performance
- Timestamp and connection status overlay

### 🌐 Web Interface
- Responsive design works on desktop, tablet, and mobile
- Real-time status updates
- Visual feedback for all controls
- Modern gradient UI with smooth animations

## 🚀 Quick Start

### Prerequisites
- Python 3.8+
- ROS2 (Humble or newer)
- OpenCV (`pip install opencv-python`)
- Flask (`pip install flask`)

### Installation

1. Clone the repository:
```bash
git clone <your-repo-url>
cd robot-web-controller
```

2. Install Python dependencies:
```bash
pip install flask opencv-python numpy pyserial
```

3. Configure your robot's IP address (default: `192.168.16.154`):
   - Edit `PI_IP` in `app_controller.py`
   - Or use the web interface settings panel

### Running the Controller

1. **Start the controller server:**
```bash
python app_controller.py
```
- Web interface available at: http://localhost:5000
- Control server port: 5000

2. **Start the video streaming server (in a new terminal):**
```bash
python app_video.py
```
- Video stream available at: http://localhost:5001
- Video server port: 5001

3. **On your robot (Raspberry Pi), start the ROS2 server:**
```bash
ros2 run your_package server.py
```

## 📁 Project Structure

```
robot-web-controller/
├── app_controller.py    # Main web controller server
├── app_video.py         # Video streaming server
├── socket_client.py     # Socket communication client
├── server.py           # ROS2 robot server (runs on Pi)
└── README.md           # This file
```

## 🎯 Usage Guide

### Manual Control
1. Select "Manual Control" mode
2. Use the on-screen directional buttons or keyboard:
   - **W/↑**: Forward
   - **S/↓**: Backward
   - **A/←**: Left
   - **D/→**: Right
   - **Space**: Stop

### Object Following
1. Select "Object Following" mode
2. Point the robot's camera at an object to track
3. The robot will:
   - Center the object in view
   - Maintain safe following distance (30-50cm)
   - Stop if object gets too close (<30cm)
   - Back up if critically close (<20cm)

### Distance Indicator
- **Green**: Safe distance (30-50cm)
- **Yellow**: Warning - too close or too far
- **Red**: Danger - very close (<20cm)

## ⚙️ Configuration

### Network Settings
Configure in the web interface or edit directly in code:

```python
# app_controller.py
pi_ip = "192.168.16.154"  # Your robot's IP
pi_port = 8888            # Control port

# app_video.py
PI_IP = "192.168.16.154"  # Your robot's IP
VIDEO_PORT = 8889         # Video streaming port
```

### Robot Parameters
Adjust in `server.py`:

```python
# Movement speeds
self.linear_speed = 0.2   # m/s
self.angular_speed = 1.0  # rad/s

# Follow mode parameters
self.min_follow_distance = 30.0   # cm
self.safe_follow_distance = 50.0  # cm
self.follow_kp = 0.003            # Turning gain
```

## 🔧 Troubleshooting

### Connection Issues
- Ensure robot and controller are on the same network
- Check firewall settings for ports 5000, 5001, 8888, 8889
- Verify robot's IP address in settings
- Use "Reconnect" button to re-establish connection

### Video Stream Not Working
- Check if video server is running (`python app_video.py`)
- Verify camera is connected to robot
- Check ROS2 camera node is publishing to `/image_raw`

### Robot Not Responding
- Verify ROS2 server is running on robot
- Check motor controller serial connection (`/dev/ttyUSB0`)
- Ensure proper ROS2 topics are being published

## 🤝 Architecture

```
┌─────────────┐     HTTP      ┌──────────────┐
│  Web Browser│◄──────────────►│ Controller   │
│             │                │ Server :5000 │
└─────────────┘                └──────┬───────┘
                                      │ Socket
┌─────────────┐     HTTP      ┌──────▼───────┐
│  Web Browser│◄──────────────►│ Video Server │
│  (Video)    │                │    :5001     │
└─────────────┘                └──────┬───────┘
                                      │ Socket
                               ┌──────▼───────┐
                               │  Robot (Pi)  │
                               │  ROS2 Server │
                               │  :8888/:8889 │
                               └──────────────┘
```

## 📊 Performance

- **Control Latency**: <100ms typical
- **Video Latency**: <200ms typical
- **Command Rate**: Up to 10 Hz continuous
- **Video Frame Rate**: ~30 FPS

## 🛡️ Safety Features

- Automatic stop on connection loss
- Command timeout protection (500ms)
- Emergency stop with ultrasonic sensors (<15cm)
- Gradual speed control based on distance
- Mode switching stops all movement

## 🔌 API Endpoints

### Controller Server (Port 5000)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Main web interface |
| `/set_mode` | POST | Change control mode (manual/follow) |
| `/move` | POST | Single movement command |
| `/move_start` | POST | Start continuous movement |
| `/move_stop` | POST | Stop movement |
| `/tracking_data` | GET | Get object tracking data |
| `/status` | GET | Get robot status |
| `/set_pi_ip` | POST | Update robot IP address |
| `/reconnect` | POST | Reconnect to robot |

### Video Server (Port 5001)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Video viewer page |
| `/video_feed` | GET | MJPEG video stream |
| `/status` | GET | Connection status |

## 📋 Requirements

### Python Packages
```
flask>=2.0.0
opencv-python>=4.5.0
numpy>=1.19.0
pyserial>=3.5
```

### ROS2 Packages
- rclpy
- std_msgs
- geometry_msgs
- sensor_msgs
- cv_bridge

## 🚧 Known Limitations

- Maximum 5 simultaneous client connections
- Video resolution fixed at 640x480
- Serial port hardcoded to `/dev/ttyUSB0`
- Single object tracking in follow mode

## 🔮 Future Enhancements

- [ ] Multiple object tracking
- [ ] Path planning and navigation
- [ ] Configurable video resolution
- [ ] Recording and playback features
- [ ] Mobile app support
- [ ] Multi-robot control

## 📝 License

MIT License - Feel free to use and modify for your projects!

## 🙏 Acknowledgments

Built with:
- ROS2 for robot control
- Flask for web framework
- OpenCV for video processing
- Modern web technologies (HTML5, CSS3, JavaScript)

---

**Need help?** Open an issue or contact the maintainers.