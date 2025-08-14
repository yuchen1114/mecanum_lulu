"""
Socket client for communicating with the robot server
IMPROVED: Better continuous movement handling and reduced timeout issues
"""

import socket
import threading
import time
import queue

class RobotSocketClient:
    def __init__(self, host='192.168.16.154', port=8888):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.running = False
        
        # Command queue for continuous commands
        self.command_queue = queue.Queue()
        self.continuous_command = None
        self.continuous_interval = 0.2  # IMPROVED: Slower interval to reduce server load
        self.continuous_thread = None
        
        # Callbacks
        self.response_callback = None
        self.connection_callback = None
        
        # Response handling
        self.last_response = None
        self.response_lock = threading.Lock()
        
        # IMPROVED: Movement state tracking
        self.current_movement = None
        self.movement_lock = threading.Lock()
    
    def set_response_callback(self, callback):
        """Set callback for command responses
        Callback signature: callback(command, response, success)
        """
        self.response_callback = callback
    
    def set_connection_callback(self, callback):
        """Set callback for connection status changes
        Callback signature: callback(connected, message)
        """
        self.connection_callback = callback
    
    def connect(self):
        """Connect to the robot server"""
        try:
            # Close existing connection if any
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
            
            # Create new socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # 5 second timeout for connection
            
            # Try to connect
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(2.0)  # IMPROVED: Longer timeout for operations
            
            self.connected = True
            self.running = True
            
            if self.connection_callback:
                self.connection_callback(True, f"Connected to {self.host}:{self.port}")
            
            return True
            
        except socket.timeout:
            self.connected = False
            if self.connection_callback:
                self.connection_callback(False, "Connection timeout")
            return False
            
        except Exception as e:
            self.connected = False
            if self.connection_callback:
                self.connection_callback(False, f"Connection failed: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from the robot server"""
        self.running = False
        self.connected = False
        
        # Stop continuous command thread
        self.stop_continuous_command()
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        if self.connection_callback:
            self.connection_callback(False, "Disconnected")
    
    def is_connected(self):
        """Check if client is connected"""
        return self.connected
    
    def send_command(self, command):
        """Send a command to the robot server and get response"""
        if not self.connected:
            if self.response_callback:
                self.response_callback(command, "Not connected", False)
            return False
        
        try:
            # Send command
            self.socket.send(command.encode('utf-8'))
            
            # Try to receive response
            try:
                response = self.socket.recv(1024).decode('utf-8')
                
                with self.response_lock:
                    self.last_response = response
                
                if self.response_callback:
                    self.response_callback(command, response, True)
                
                return True
                
            except socket.timeout:
                # Timeout is OK for some commands
                if self.response_callback:
                    self.response_callback(command, "Timeout", True)
                return True
                
        except ConnectionResetError:
            self.connected = False
            if self.connection_callback:
                self.connection_callback(False, "Connection lost")
            if self.response_callback:
                self.response_callback(command, "Connection lost", False)
            return False
            
        except Exception as e:
            if self.response_callback:
                self.response_callback(command, f"Error: {str(e)}", False)
            return False
    
    def start_continuous_command(self, command, interval=0.2):
        """Start sending a command continuously at specified interval"""
        self.stop_continuous_command()
        
        self.continuous_command = command
        self.continuous_interval = max(0.1, interval)  # IMPROVED: Minimum 100ms interval
        
        # IMPROVED: Track current movement direction
        if "MOVE:" in command:
            direction = command.split("MOVE:")[1]
            with self.movement_lock:
                self.current_movement = direction
        
        self.continuous_thread = threading.Thread(
            target=self._continuous_sender,
            daemon=True
        )
        self.continuous_thread.start()
    
    def stop_continuous_command(self):
        """Stop sending continuous commands and send explicit stop"""
        self.continuous_command = None
        
        # IMPROVED: Send explicit stop command when stopping continuous movement
        with self.movement_lock:
            if self.current_movement and self.current_movement != "stop":
                self.send_command("MOVE:stop")
                self.current_movement = "stop"
        
        if self.continuous_thread:
            self.continuous_thread.join(timeout=0.5)
            self.continuous_thread = None
    
    def _continuous_sender(self):
        """Thread function for sending continuous commands"""
        while self.continuous_command and self.connected:
            self.send_command(self.continuous_command)
            time.sleep(self.continuous_interval)
    
    def get_last_response(self):
        """Get the last response received"""
        with self.response_lock:
            return self.last_response
    
    def ping(self):
        """Send a ping command to test connection"""
        if self.send_command("PING"):
            with self.response_lock:
                return self.last_response == "PONG"
        return False
    
    def reconnect(self):
        """Attempt to reconnect to the server"""
        self.disconnect()
        time.sleep(0.5)
        return self.connect()
    
    def set_host(self, host, port=None):
        """Change the host and optionally port"""
        self.host = host
        if port:
            self.port = port
        
        # If we were connected, reconnect to new host
        if self.connected:
            return self.reconnect()
        return True
    
    def get_current_movement(self):
        """Get current movement direction"""
        with self.movement_lock:
            return self.current_movement
    
    def send_single_move(self, direction):
        """Send a single movement command (for momentary actions)"""
        return self.send_command(f"MOVE:{direction}")

# Example usage
if __name__ == "__main__":
    # Create client
    client = RobotSocketClient('192.168.16.154', 8888)
    
    # Set up callbacks
    def on_response(cmd, resp, success):
        # IMPROVED: Less verbose logging for movement commands
        if not cmd.startswith("MOVE:") or not success:
            print(f"Command: {cmd} -> Response: {resp} (Success: {success})")
    
    def on_connection(connected, message):
        print(f"Connection: {connected} - {message}")
    
    client.set_response_callback(on_response)
    client.set_connection_callback(on_connection)
    
    # Connect
    if client.connect():
        print("Connected successfully!")
        
        # Test commands
        client.send_command("PING")
        time.sleep(0.5)
        
        client.send_command("STATUS")
        time.sleep(0.5)
        
        client.send_command("MODE:manual")
        time.sleep(0.5)
        
        # Test continuous movement with improved handling
        print("Starting continuous forward movement...")
        client.start_continuous_command("MOVE:forward", 0.2)  # 200ms interval
        time.sleep(3)
        
        print("Switching to backward movement...")
        client.start_continuous_command("MOVE:backward", 0.2)
        time.sleep(2)
        
        print("Stopping movement...")
        client.stop_continuous_command()
        
        time.sleep(1)
        client.disconnect()
    else:
        print("Failed to connect")