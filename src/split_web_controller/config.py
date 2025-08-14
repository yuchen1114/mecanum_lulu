# Robot Controller Configuration
# Update these settings according to your setup

# Raspberry Pi Network Settings
PI_IP = "192.168.16.154"  # My Pi's IP address
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
