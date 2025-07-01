#!/usr/bin/env python3
"""
Raspberry Pi Server for Robot Controller
This script runs on the Raspberry Pi and listens for commands from the web controller.
"""

import socket
import threading
import time
import json
from datetime import datetime

# Configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 8888
MAX_CONNECTIONS = 5

class RobotServer:
    def __init__(self):
        self.current_mode = "manual"
        self.is_moving = False
        self.last_command = None
        self.command_count = 0
        self.start_time = time.time()
        
        # Initialize GPIO or motor control here if needed
        self.setup_hardware()
    
    def setup_hardware(self):
        """
        Initialize hardware connections (GPIO, motors, sensors, etc.)
        Add your specific hardware initialization code here.
        """
        print("🔧 Initializing hardware...")
        
        # Example GPIO setup (uncomment and modify as needed):
        # import RPi.GPIO as GPIO
        # GPIO.setmode(GPIO.BCM)
        # 
        # # Motor pins
        # self.motor_pins = {
        #     'left_forward': 18,
        #     'left_backward': 19,
        #     'right_forward': 20,
        #     'right_backward': 21
        # }
        # 
        # for pin in self.motor_pins.values():
        #     GPIO.setup(pin, GPIO.OUT)
        #     GPIO.output(pin, GPIO.LOW)
        
        print("✅ Hardware initialized")
    
    def handle_mode_command(self, mode):
        """Handle mode switching commands"""
        valid_modes = ['manual', 'auto', 'follow']
        
        if mode in valid_modes:
            old_mode = self.current_mode
            self.current_mode = mode
            
            print(f"🔄 Mode changed: {old_mode} → {mode}")
            
            # Stop any current movement when switching modes
            if self.is_moving:
                self.stop_motors()
            
            # Mode-specific initialization
            if mode == 'auto':
                self.start_auto_mode()
            elif mode == 'follow':
                self.start_follow_mode()
            elif mode == 'manual':
                self.stop_auto_behaviors()
            
            return f"MODE_OK:{mode}"
        else:
            return f"MODE_ERROR:Invalid mode {mode}"
    
    def handle_move_command(self, direction):
        """Handle movement commands"""
        if self.current_mode != 'manual':
            return f"MOVE_ERROR:Not in manual mode (current: {self.current_mode})"
        
        valid_directions = ['forward', 'backward', 'left', 'right', 'stop']
        
        if direction not in valid_directions:
            return f"MOVE_ERROR:Invalid direction {direction}"
        
        print(f"🚗 Moving: {direction}")
        
        # Execute movement
        if direction == 'forward':
            self.move_forward()
        elif direction == 'backward':
            self.move_backward()
        elif direction == 'left':
            self.turn_left()
        elif direction == 'right':
            self.turn_right()
        elif direction == 'stop':
            self.stop_motors()
        
        return f"MOVE_OK:{direction}"
    
    def move_forward(self):
        """Move robot forward"""
        self.is_moving = True
        print("⬆️ Moving forward")
        
        # Add your motor control code here
        # Example:
        # GPIO.output(self.motor_pins['left_forward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['right_forward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['left_backward'], GPIO.LOW)
        # GPIO.output(self.motor_pins['right_backward'], GPIO.LOW)
    
    def move_backward(self):
        """Move robot backward"""
        self.is_moving = True
        print("⬇️ Moving backward")
        
        # Add your motor control code here
        # Example:
        # GPIO.output(self.motor_pins['left_backward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['right_backward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['left_forward'], GPIO.LOW)
        # GPIO.output(self.motor_pins['right_forward'], GPIO.LOW)
    
    def turn_left(self):
        """Turn robot left"""
        self.is_moving = True
        print("⬅️ Turning left")
        
        # Add your motor control code here
        # Example:
        # GPIO.output(self.motor_pins['left_backward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['right_forward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['left_forward'], GPIO.LOW)
        # GPIO.output(self.motor_pins['right_backward'], GPIO.LOW)
    
    def turn_right(self):
        """Turn robot right"""
        self.is_moving = True
        print("➡️ Turning right")
        
        # Add your motor control code here
        # Example:
        # GPIO.output(self.motor_pins['left_forward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['right_backward'], GPIO.HIGH)
        # GPIO.output(self.motor_pins['left_backward'], GPIO.LOW)
        # GPIO.output(self.motor_pins['right_forward'], GPIO.LOW)
    
    def stop_motors(self):
        """Stop all robot movement"""
        self.is_moving = False
        print("🛑 Stopping motors")
        
        # Add your motor control code here
        # Example:
        # for pin in self.motor_pins.values():
        #     GPIO.output(pin, GPIO.LOW)
    
    def start_auto_mode(self):
        """Initialize autonomous mode"""
        print("🤖 Starting autonomous mode")
        # Add autonomous behavior code here
        # This could include obstacle avoidance, path following, etc.
    
    def start_follow_mode(self):
        """Initialize follow mode"""
        print("👁️ Starting follow mode")
        # Add object tracking/following code here
        # This could use camera input to follow objects or people
    
    def stop_auto_behaviors(self):
        """Stop any autonomous behaviors"""
        print("🛑 Stopping autonomous behaviors")
        self.stop_motors()
    
    def get_status(self):
        """Get current robot status"""
        uptime = time.time() - self.start_time
        return {
            'mode': self.current_mode,
            'is_moving': self.is_moving,
            'last_command': self.last_command,
            'command_count': self.command_count,
            'uptime': round(uptime, 1),
            'timestamp': datetime.now().isoformat()
        }
    
    def process_command(self, command_str):
        """Process incoming command and return response"""
        try:
            command_str = command_str.strip()
            self.last_command = command_str
            self.command_count += 1
            
            print(f"📥 Command #{self.command_count}: {command_str}")
            
            if ':' in command_str:
                command_type, command_value = command_str.split(':', 1)
                
                if command_type == 'MODE':
                    return self.handle_mode_command(command_value)
                elif command_type == 'MOVE':
                    return self.handle_move_command(command_value)
                else:
                    return f"ERROR:Unknown command type {command_type}"
            
            elif command_str == 'STATUS':
                status = self.get_status()
                return f"STATUS_OK:{json.dumps(status)}"
            
            elif command_str == 'PING':
                return "PONG"
            
            else:
                return f"ERROR:Invalid command format: {command_str}"
                
        except Exception as e:
            error_msg = f"ERROR:Exception processing command: {str(e)}"
            print(f"❌ {error_msg}")
            return error_msg

def handle_client(client_socket, client_address, robot):
    """Handle individual client connection"""
    print(f"🔗 New connection from {client_address}")
    
    try:
        while True:
            # Receive data from client
            data = client_socket.recv(1024)
            if not data:
                break
            
            command = data.decode('utf-8')
            
            # Process command
            response = robot.process_command(command)
            
            # Send response back to client
            client_socket.send(response.encode('utf-8'))
            
            print(f"📤 Response to {client_address}: {response}")
            
    except ConnectionResetError:
        print(f"🔌 Connection reset by {client_address}")
    except Exception as e:
        print(f"❌ Error handling client {client_address}: {e}")
    finally:
        client_socket.close()
        print(f"❌ Connection closed with {client_address}")

def main():
    """Main server function"""
    print("🤖 Robot Controller Server Starting...")
    print("=" * 50)
    
    # Create robot instance
    robot = RobotServer()
    
    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        # Bind socket to address
        server_socket.bind((HOST, PORT))
        server_socket.listen(MAX_CONNECTIONS)
        
        print(f"🌐 Server listening on {HOST}:{PORT}")
        print(f"📡 Max connections: {MAX_CONNECTIONS}")
        print(f"⏰ Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 50)
        print("🔄 Waiting for connections...")
        
        while True:
            try:
                # Accept incoming connection
                client_socket, client_address = server_socket.accept()
                
                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=handle_client,
                    args=(client_socket, client_address, robot),
                    daemon=True
                )
                client_thread.start()
                
            except KeyboardInterrupt:
                print("\n🛑 Server shutdown requested")
                break
            except Exception as e:
                print(f"❌ Error accepting connection: {e}")
                
    except Exception as e:
        print(f"❌ Server error: {e}")
    finally:
        # Cleanup
        print("🧹 Cleaning up...")
        robot.stop_motors()
        server_socket.close()
        
        # GPIO cleanup if using RPi.GPIO
        # import RPi.GPIO as GPIO
        # GPIO.cleanup()
        
        print("✅ Server stopped")

if __name__ == "__main__":
    main()