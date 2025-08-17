# Mecanum Autonomous Vehicle

A three-wheeled omnidirectional autonomous vehicle built from scratch, featuring complete translation and rotation capabilities in any direction. This project encompasses hardware design, firmware development, and software implementation as a comprehensive learning platform for robotics and autonomous systems.

<p align="center">
  <img src="_image/front.PNG" alt="Car Image 1" width="400"/>
  <img src="_image/hind.PNG" alt="Car Image 2" width="400"/>
</p>
<p align="center">
  <img src="_image/top.PNG" alt="Car Image 3" width="400"/>
  <img src="_image/bottom.PNG" alt="Car Image 4" width="400"/>
</p>

## 🎯 Features

- **Omnidirectional Movement**: Full 360° translation and rotation using mecanum wheel configuration
- **Multiple Control Modes**:
  - Keyboard control for direct input
  - Web-based controller with intuitive interface
  - Manual mode for precise control
  - Follow mode with object tracking
- **Computer Vision**: YOLOv8-based object detection and tracking with custom algorithms
- **Real-time Video Streaming**: Live video feed accessible through web interface
- **Full Stack Implementation**: Hardware, firmware, and software built from scratch

<!-- Insert demo video here -->
[//]: # (Add video with: [![Demo Video](path/to/video-thumbnail.jpg)](video-url))
[//]: # (Or embed GIF: ![Demo](path/to/demo.gif))

## 📁 Project Structure

```
mecanum_lulu/
├── build/              # Build files and compiled outputs
├── docs/               # Documentation and design specifications
├── hardware/           # Hardware schematics and PCB designs
├── install/            # Installation scripts and dependencies
├── log/                # YOLO model and its performance
├── src/                # Source code
│   ├── firmware/       # C++ microcontroller code
│   ├── software/       # Python control and vision systems
│   └── web/           # HTML/CSS/JS web controller interface
└── README.md          # This file
```

## 🚀 Getting Started

### Clone the Repository

```bash
git clone https://github.com/yourusername/mecanum_lulu.git
cd mecanum_lulu
```

## ⚙️ Firmware Setup

The firmware is uploaded using Arduino IDE with the provided `.ino` file.

### Requirements
- Arduino IDE 1.8+ or Arduino IDE 2.0
- USB cable for microcontroller connection

### Upload Instructions
1. Open `src/firmware/mecanum_controller.ino` in Arduino IDE
2. Select the correct board type from Tools menu
3. Select the appropriate COM port
4. Click Upload button

## 💻 Software Setup

The software stack is deployed using ROS2 (Robot Operating System 2).

### Requirements
- ROS2 Humble or later (this project uses jazzy)
- Python 3.8+
- YOLOv8

### Deployment
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Launch the system
ros2 run [package_name] [node_name]
```

## 🔧 Hardware Components

<!-- Insert hardware assembly image here -->
[//]: # (Add image of the assembled vehicle: ![Hardware Assembly](path/to/hardware.jpg))

### Drive System
- **Motors**: ASLONG JGB37-520 12V DC Worm Gear Motor
- **Wheels**: Mecanum wheels (3x)
- **Motor Drivers**: L298N

### Control System
- **Microcomputer**: Raspberry Pi 5
- **Microcontroller**: ESP32 NodeMCU-32S
- **Power Supply**: Lithium Battery
- **DC Buck Converter Module**: XL4015

### Sensors & Vision
- **Camera**: DTAudio 1080P webcam
- **Ultrasonic sensor**: HC-SR04

### Frame & Structure
- **Chassis Material**: Aluminum

<!-- Insert component layout diagram here -->
[//]: # (Add diagram showing component placement: ![Component Layout](path/to/layout.png))

## 💻 Usage

### Keyboard Control

Run the keyboard controller:
```bash
ros2 run keyboard keyboard_node
ros2 run communication UART
#-optional
ros2 topic echo /cmd_vel
```

Controls:
- `W/A/S/D` - Forward/Left/Backward/Right translation
- `Q/E` - Rotate counterclockwise/clockwise
- `J` - Emergency stop
- `P` - Exit

### Web Controller

1. Start the web server:
```bash
ros2 run server server_node
ros2 run communication UART
ros2 run usb_cam camera
ros2 run image_processing YOLO
#-optional
ros2 topic echo /cmd_vel
```

2. Run app in PC, open browser and navigate to:
```bash
python app_controller.py
python app_video.py
```
```
http://localhost:5000
http://localhost:5001
```

3. Select control mode:
   - **Manual Mode**: Use on-screen arrow keys
   - **Follow Mode**: Object tracking based on trained yolov8 model and data from ultrasonic sensor

### Object Tracking & Sensor Fusion

The follow mode uses YOLOv8 for object detection with a custom ROS2-based tracking algorithm:

```python
# Core tracking algorithm features:
- YOLO model for object detection (custom trained model)
- Center-line tracking with error calculation
- Confidence threshold filtering (0.70)
- Real-time visual feedback with bounding boxes
- ROS2 integration for control system communication
- Ultrasonic sensor integration for distance tracking
```

**How it works:**
1. **Detection**: YOLO model processes video frames to detect objects
2. **Tracking**: Calculates center position of detected object
3. **Error Calculation**: Computes horizontal offset from frame center
4. **Control Signal**: Publishes error value to `/tracker_data` topic
5. **Visual Feedback**: Displays annotated frame with bounding boxes and center line
6. **Emergency stop**: Stops vehicle when distance smaller than 20cm to prevent crashing

**Key Features:**
- Tracks first detected object above confidence threshold
- Automatic boundary limit handling
- Real-time performance with ROS2 QoS optimization
- Visual debugging with OpenCV display

<!-- Insert tracking demonstration image/gif here -->
[//]: # (Add image showing tracking in action: ![Tracking Demo](path/to/tracking-demo.gif))

## 🔧 Technical Specifications

### Technical Overview
- **Software**: ROS2-based control system with Python
- **Firmware**: C++ on Arduino platform
- **Vision System**: YOLOv8 for object detection and tracking
- **Web Interface**: HTML/CSS/JavaScript for remote control
- **Wheel Configuration**: 3-wheel mecanum setup for omnidirectional movement

## 🎮 Control Modes

### Manual Mode
Direct control through keyboard or web interface with real-time response. Supports:
- Linear movement in any direction
- Rotation while stationary

<!-- Insert manual control demo here -->
[//]: # (Add gif showing manual control: ![Manual Control](path/to/manual-control.gif))

### Follow Mode
Autonomous tracking using computer vision:
- YOLOv8 object detection with custom-trained models
- Center-line tracking algorithm for smooth following
- Confidence-based filtering (threshold: 0.70)
- Error-based control signal generation
- Visual feedback with bounding boxes and tracking indicators

<!-- Insert follow mode demo here -->
[//]: # (Add gif showing follow mode: ![Follow Mode](path/to/follow-mode.gif))

## 📹 Video Streaming

Access real-time video feed through the web controller:
- Low latency streaming via ROS2 image topics
- Object detection overlay with bounding boxes
- Center-line reference for tracking alignment
- FPS and confidence score display

<!-- Insert web interface screenshot here -->
[//]: # (Add screenshot of web controller: ![Web Interface](path/to/web-interface.png))


## 📊 Performance

- Maximum Speed: [TBD]
- Turning Radius: Zero (omnidirectional)
- Battery Life: [TBD]
- Video Latency: <100ms (local network)
- Object Tracking FPS: 30+ fps

## 📝 Future Improvements

- [ ] Implement SLAM for autonomous navigation
- [ ] Add gamepad/joystick support
- [ ] Add IMU support
- [ ] Implement path planning algorithms
- [ ] Add multiple camera support
- [ ] Develop mobile app controller

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- YOLOv8 team for object detection framework
- OpenCV community for computer vision tools

## 📧 Contact

- Project Maintainer: [Lulu]

---

*This project is a learning journey from hardware to software, demonstrating the complete development cycle of an autonomous robotic system.*