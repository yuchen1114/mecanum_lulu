#!/usr/bin/env python3
"""
Setup script for Robot Controller Web App
Run this script to set up the project structure and install dependencies.
"""

import os
import sys
import subprocess
import shutil

def create_directory_structure():
    """Create the project directory structure"""
    print("📁 Creating directory structure...")
    
    directories = [
        'robot_controller',
        'robot_controller/templates',
        'robot_controller/static',
        'robot_controller/static/css',
        'robot_controller/static/js'
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"   ✅ Created: {directory}")

def install_dependencies():
    """Install Python dependencies"""
    print("\n📦 Installing Python dependencies...")
    
    try:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'flask'])
        print("   ✅ Flask installed successfully")
    except subprocess.CalledProcessError as e:
        print(f"   ❌ Error installing dependencies: {e}")
        return False
    
    return True

def create_config_file():
    """Create a configuration file"""
    config_content = """# Robot Controller Configuration
# Update these settings according to your setup

# Raspberry Pi Network Settings
PI_IP = "192.168.1.100"  # Change this to your Pi's IP address
PI_PORT = 8888

# Web Server Settings
WEB_HOST = "0.0.0.0"  # Listen on all interfaces
WEB_PORT = 5000
DEBUG = True  # Set to False in production

# Robot Settings
DEFAULT_MODE = "manual"
MOVEMENT_TIMEOUT = 5  # seconds

# Hardware Settings (for Raspberry Pi)
# Uncomment and modify these if using GPIO
# MOTOR_PINS = {
#     'left_forward': 18,
#     'left_backward': 19,
#     'right_forward': 20,
#     'right_backward': 21
# }
"""
    
    with open('robot_controller/config.py', 'w') as f:
        f.write(config_content)
    
    print("   ✅ Created config.py")

def create_readme():
    """Create a detailed README file"""
    readme_content = """# Robot Controller Web App

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
"""
    
    with open('robot_controller/README.md', 'w', encoding='utf-8') as f:
        f.write(readme_content)
    
    print("   ✅ Created README.md")

def main():
    """Main setup function"""
    print("🤖 Robot Controller Setup")
    print("=" * 40)
    
    # Create directory structure
    create_directory_structure()
    
    # Install dependencies
    if not install_dependencies():
        print("\n❌ Setup failed due to dependency installation error")
        return
    
    # Create configuration file
    create_config_file()
    
    # Create README
    create_readme()
    
    print("\n" + "=" * 40)
    print("✅ Setup completed successfully!")
    print("\n📋 Next steps:")
    print("1. Copy the generated files to robot_controller/ directory")
    print("2. Update PI_IP in config.py with your Raspberry Pi's IP")
    print("3. Copy server.py to your Raspberry Pi")
    print("4. Run server.py on the Pi: python3 server.py")
    print("5. Run app.py on your computer: python app.py")
    print("6. Open http://localhost:5000 in your browser")
    print("\n🎉 Happy robot controlling!")

if __name__ == "__main__":
    main()