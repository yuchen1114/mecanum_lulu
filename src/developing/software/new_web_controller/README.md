# Enhanced Robot Controller Setup Guide

## Overview
This enhanced robot controller system provides:
- **Real-time video streaming** from the robot's camera
- **Manual control** with keyboard and mouse/touch input
- **Gyro control** using phone orientation
- **Autonomous navigation** using ultrasonic sensors
- **Object following** mode with color-based tracking
- **Web-based interface** accessible from any device

## System Architecture

### On Raspberry Pi 5:
1. **enhanced_server.py** - Main ROS2 node that handles all modes and communications
2. **ultrasonic_sensors.py** - Reads HC-SR04 sensors and publishes to ROS2
3. **object_tracker.py** - Tracks colored objects for follow mode
4. **Camera node** - Publishes video stream (using v4l2_camera or usb_cam)

### On PC:
1. **app.py** - Flask web server
2. **index.html** - Web interface
3. **socket_client.py** - Socket communication handler
4. **config.py** - Configuration file

## Installation

### Prerequisites

#### Raspberry Pi:
```bash
# Install ROS2 Humble
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Install Python packages
pip3 install opencv-python numpy RPi.GPIO

# Install camera package
sudo apt install ros-humble-v4l2-camera
# OR
sudo apt install ros-humble-usb-cam
```

#### PC:
```bash
# Install Python packages
pip3 install flask opencv-python numpy pillow
```

## Setup Instructions

### 1. Configure Network
Make sure your PC and Raspberry Pi are on the same network. Update the IP address in `config.py`:
```python
PI_IP = "192.168.16.154"  # Your Pi's IP address
```

### 2. Set Up Raspberry Pi

1. **Create ROS2 workspace:**
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
```

2. **Create package:**
```bash
ros2 pkg create --build-type ament_python robot_controller
```

3. **Copy scripts to package:**
```bash
cp enhanced_server.py ~/robot_ws/src/robot_controller/robot_controller/
cp ultrasonic_sensors.py ~/robot_ws/src/robot_controller/robot_controller/
cp object_tracker.py ~/robot_ws/src/robot_controller/robot_controller/
```

4. **Update setup.py:**
Add entry points for your scripts:
```python
entry_points={
    'console_scripts': [
        'enhanced_server = robot_controller.enhanced_server:main',
        'ultrasonic_sensors = robot_controller.ultrasonic_sensors:main',
        'object_tracker = robot_controller.object_tracker:main',
    ],
},
```

5. **Build workspace:**
```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

### 3. Set Up PC

1. **Create project directory:**
```bash
mkdir robot_controller_web
cd robot_controller_web
```

2. **Copy files:**
- `app.py`
- `socket_client.py`
- `config.py`
- Create `templates/` directory and copy `index.html`

3. **Update configuration:**
Edit `config.py` with your Pi's IP address.

## Running the System

### On Raspberry Pi:

#### Option 1: Using Launch File
```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch robot_controller robot_launch.py
```

#### Option 2: Manual Start
```bash
# Terminal 1 - Main server
ros2 run robot_controller enhanced_server

# Terminal 2 - Ultrasonic sensors
ros2 run robot_controller ultrasonic_sensors

# Terminal 3 - Camera (choose one)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
# OR
ros2 run usb_cam usb_cam_node_exe

# Terminal 4 - Object tracker (optional)
ros2 run robot_controller object_tracker
```

### On PC:
```bash
cd robot_controller_web
python3 app.py
```

Then open a web browser and go to: `http://localhost:5000`

## Usage Guide

### Control Modes

1. **Manual Mode**
   - Use arrow keys or WASD for movement
   - Click/touch movement buttons
   - Hold buttons for continuous movement

2. **Auto Mode**
   - Robot navigates autonomously using ultrasonic sensors
   - Avoids obstacles and explores environment
   - Displays sensor readings in real-time

3. **Follow Mode**
   - Tracks and follows colored objects
   - Default: tracks red objects
   - Configure color in object_tracker.py

4. **Gyro Mode**
   - Tilt your phone to control robot
   - Forward/back: tilt phone forward/backward
   - Left/right: tilt phone left/right
   - Requires permission on iOS devices

### Video Stream
- Live video displayed for all modes
- 640x480 resolution at 30 FPS
- Shows connection status and FPS

### Troubleshooting

1. **Video not showing:**
   - Check camera connection: `ls /dev/video*`
   - Test camera: `v4l2-ctl --list-devices`
   - Ensure camera node is running

2. **Connection failed:**
   - Verify Pi IP address: `hostname -I`
   - Check firewall settings
   - Ensure ports 8888 and 8889 are open

3. **Ultrasonic sensors not working:**
   - Check GPIO connections
   - Verify pin numbers in ultrasonic_sensors.py
   - Run `gpio readall` to check pin states

4. **Gyro mode not working:**
   - HTTPS required for device orientation API
   - Click "Enable Gyro Control" button
   - Allow permission when prompted

## Customization

### Change Movement Speed
In `enhanced_server.py`:
```python
self.linear_speed = 0.2  # m/s
self.angular_speed = 1.0  # rad/s
```

### Change Tracking Color
In `object_tracker.py`:
```python
self.declare_parameter('target_color', 'red')  # Change to 'green', 'blue', 'yellow'
```

### Adjust Obstacle Detection
In `enhanced_server.py`:
```python
obstacle_threshold = 0.3  # meters
clear_threshold = 0.8     # meters
```

### Modify Video Resolution
In launch file or camera command:
```python
'image_size': [640, 480],  # Change to desired resolution
'framerate': 30.0,         # Change FPS
```

## Safety Notes

- Always have a way to stop the robot (STOP button or spacebar)
- Test in a safe, open area first
- Monitor sensor readings during autonomous modes
- Ensure proper power supply for all components
- Add emergency stop hardware if needed

## Future Enhancements

1. **SLAM Integration** - Add mapping and localization
2. **Path Planning** - Implement A* or RRT for navigation
3. **Machine Learning** - Use YOLO or TensorFlow for object detection
4. **Voice Control** - Add speech recognition
5. **Multiple Camera Support** - Add rear/side cameras
6. **Battery Monitoring** - Display power levels
7. **GPS Integration** - Outdoor navigation
8. **Gamepad Support** - Xbox/PlayStation controller input

## Support

For issues or questions:
1. Check ROS2 logs: `ros2 topic echo /rosout`
2. Monitor topics: `ros2 topic list`
3. Check node status: `ros2 node list`
4. View sensor data: `ros2 topic echo /ultrasonic_array`

Happy robot controlling! 🤖