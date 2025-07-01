# Robot Controller Web App

A Python Flask web application for controlling a robot over Wi-Fi using socket connections.

## Quick Start

### 1. Setup the Web Controller (Local Computer)

```bash
# Install dependencies
pip install flask

# Run the web server
python app.py
```

The web interface will be available at: http://localhost:5000

### 2. Setup the Raspberry Pi Server

Copy `server.py` to your Raspberry Pi and run:

```bash
python3 server.py
```

### 3. Configure Network Settings

1. Find your Raspberry Pi's IP address:
   ```bash
   hostname -I
   ```

2. Update the Pi IP in the web interface settings or modify `config.py`

## Project Structure

```
robot_controller/
├── app.py              # Flask web server
├── socket_client.py    # Communication with Pi
├── server.py          # Raspberry Pi server
├── config.py          # Configuration settings
├── requirements.txt   # Python dependencies
├── templates/
│   └── index.html     # Web interface
└── README.md          # This file
```

## Features

- **Three Control Modes:**
  - Manual: Direct control with buttons/keyboard
  - Auto: Autonomous navigation
  - Follow: Object tracking and following

- **Movement Controls:**
  - Forward/Backward/Left/Right movement
  - Emergency stop
  - Keyboard controls (WASD/Arrow keys)

- **Real-time Status:**
  - Connection status with Raspberry Pi
  - Current control mode
  - Robot movement state

## Keyboard Controls (Manual Mode)

- **W / Up Arrow**: Forward
- **S / Down Arrow**: Backward  
- **A / Left Arrow**: Left
- **D / Right Arrow**: Right
- **Space**: Stop

## Hardware Setup (Raspberry Pi)

The `server.py` file includes commented GPIO code for motor control. Uncomment and modify the hardware setup sections according to your robot's wiring:

```python
# Example motor pin configuration
motor_pins = {
    'left_forward': 18,
    'left_backward': 19,
    'right_forward': 20,
    'right_backward': 21
}
```

## Network Configuration

Make sure both devices are on the same network:

1. **Find Pi IP address:**
   ```bash
   hostname -I
   ```

2. **Test connection:**
   ```bash
   ping [PI_IP_ADDRESS]
   ```

3. **Update IP in web interface or config.py**

## Troubleshooting

### Connection Issues
- Verify both devices are on same network
- Check firewall settings on Raspberry Pi
- Ensure server.py is running on Pi before starting web controller

### Movement Issues
- Confirm robot is in manual mode
- Check hardware connections on Pi
- Verify GPIO pin assignments match your wiring

### Web Interface Issues
- Try different browser
- Check Flask server logs for errors
- Ensure port 5000 is not in use

## License

This project is open source and available under the MIT License.
