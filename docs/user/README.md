# Robot Control System

A ROS2-based robot control system supporting multiple operation modes including manual control, autonomous operation, and follow functionality.

## Prerequisites

- ROS2 (tested with appropriate distribution)
- Required packages: `core_msg`, `keyboard`, `server`

## Quick Start

All modes require the teleporter node to be running:

```bash
ros2 run core_msg teleporter
```

## Operation Modes

### Manual Mode

Control the robot manually using different input methods.

#### Option 1: Keyboard Control
```bash
# Terminal 1
ros2 run core_msg teleporter

# Terminal 2
ros2 run keyboard keyboard_node
```

#### Option 2: Web Controller
```bash
# Terminal 1
ros2 run core_msg teleporter

# Terminal 2
ros2 run server server
```
Then navigate to the web interface and select **MANUAL** mode.

#### Option 3: Joystick Control
```bash
# Terminal 1
ros2 run core_msg teleporter

# Terminal 2
# Joystick implementation - TBD
```

### Auto Mode

Autonomous operation mode for predefined tasks.

```bash
# Terminal 1
ros2 run core_msg teleporter

# Terminal 2
ros2 run server server
```
Navigate to the web interface and select **AUTO** mode.

*Note: Autonomous tasks are currently under development (TBD)*

### Follow Mode

Robot follows a target or predefined path.

```bash
# Terminal 1
ros2 run core_msg teleporter

# Terminal 2
ros2 run server server
```
Navigate to the web interface and select **FOLLOW** mode.

*Note: Follow functionality is currently under development (TBD)*

## Usage Notes

- Always start the `teleporter` node first before launching any control interface
- For web-based modes, ensure the server is running and accessible
- Multiple terminals may be required for full operation

## Development Status

- ✅ Manual keyboard control - Implemented
- ✅ Manual web control - Implemented  
- 🚧 Joystick control - To Be Determined
- 🚧 Auto mode tasks - To Be Determined
- 🚧 Follow mode tasks - To Be Determined

## Troubleshooting

If you encounter issues:

1. Verify all required ROS2 packages are installed
2. Check that the `teleporter` node is running before starting control interfaces
3. Ensure proper ROS2 environment setup (`source /opt/ros/<distro>/setup.bash`)

## Contributing

Please refer to the project documentation for contribution guidelines and development setup instructions.