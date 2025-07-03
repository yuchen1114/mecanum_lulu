import socket
import time
import json
import threading
from typing import Optional, Callable

class RobotSocketClient:
    """
    Long-lived socket client for robot communication
    Supports continuous commands and connection management
    """
    
    def __init__(self, pi_ip: str, pi_port: int, timeout: int = 5):
        self.pi_ip = pi_ip
        self.pi_port = pi_port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.running = False
        self.command_queue = []
        self.response_callback: Optional[Callable] = None
        self.connection_callback: Optional[Callable] = None
        self.lock = threading.Lock()
        
        # Command sending thread
        self.command_thread = None
        
        # Continuous command state
        self.continuous_command = None
        self.continuous_active = False
        self.continuous_interval = 0.1  # 100ms interval for continuous commands
    
    def set_response_callback(self, callback: Callable):
        """Set callback for command responses"""
        self.response_callback = callback
    
    def set_connection_callback(self, callback: Callable):
        """Set callback for connection status changes"""
        self.connection_callback = callback
    
    def connect(self) -> bool:
        """Establish connection to Pi"""
        try:
            if self.socket:
                self.disconnect()
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.pi_ip, self.pi_port))
            
            # Set to non-blocking for continuous operation
            self.socket.setblocking(False)
            
            self.connected = True
            self.running = True
            
            # Start command processing thread
            self.command_thread = threading.Thread(target=self._command_processor, daemon=True)
            self.command_thread.start()
            
            if self.connection_callback:
                self.connection_callback(True, "Connected")
            
            print(f"✅ Connected to Pi at {self.pi_ip}:{self.pi_port}")
            return True
            
        except Exception as e:
            self.connected = False
            if self.connection_callback:
                self.connection_callback(False, f"Connection failed: {str(e)}")
            print(f"❌ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Pi"""
        self.running = False
        self.continuous_active = False
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        self.connected = False
        
        if self.connection_callback:
            self.connection_callback(False, "Disconnected")
        
        print("🔌 Disconnected from Pi")
    
    def is_connected(self) -> bool:
        """Check if connected to Pi"""
        return self.connected and self.socket is not None
    
    def send_command(self, command: str) -> bool:
        """Send a single command (non-blocking)"""
        if not self.is_connected():
            return False
        
        with self.lock:
            self.command_queue.append({
                'command': command,
                'timestamp': time.time(),
                'type': 'single'
            })
        
        return True
    
    def start_continuous_command(self, command: str, interval: float = 0.1):
        """Start sending a command continuously"""
        with self.lock:
            self.continuous_command = command
            self.continuous_interval = interval
            self.continuous_active = True
        
        print(f"🔄 Started continuous command: {command}")
    
    def stop_continuous_command(self):
        """Stop sending continuous commands"""
        with self.lock:
            self.continuous_active = False
            self.continuous_command = None
        
        print("⏹️ Stopped continuous command")
    
    def _command_processor(self):
        """Background thread to process commands"""
        last_continuous_time = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                # Process continuous commands
                if self.continuous_active and self.continuous_command:
                    if current_time - last_continuous_time >= self.continuous_interval:
                        self._send_raw_command(self.continuous_command)
                        last_continuous_time = current_time
                
                # Process single commands
                with self.lock:
                    if self.command_queue:
                        cmd_data = self.command_queue.pop(0)
                        self._send_raw_command(cmd_data['command'])
                
                time.sleep(0.01)  # Small delay to prevent busy waiting
                
            except Exception as e:
                print(f"❌ Error in command processor: {e}")
                if not self.running:
                    break
    
    def _send_raw_command(self, command: str):
        """Send raw command to Pi"""
        if not self.socket:
            return
        
        try:
            # Send command
            message = command.encode('utf-8')
            self.socket.send(message)
            
            # Try to receive response (non-blocking)
            try:
                response = self.socket.recv(1024).decode('utf-8')
                if self.response_callback:
                    self.response_callback(command, response, True)
            except socket.error:
                # No response available (non-blocking), that's OK
                pass
                
        except socket.error as e:
            print(f"❌ Socket error: {e}")
            self.connected = False
            if self.connection_callback:
                self.connection_callback(False, f"Socket error: {e}")
    
    def ping(self) -> bool:
        """Send ping to check connection"""
        return self.send_command("PING")
    
    def get_status(self) -> bool:
        """Request status from Pi"""
        return self.send_command("STATUS")

# Backward compatibility functions
def send_command(pi_ip, pi_port, command, timeout=5):
    """
    Send a single command (legacy function for backward compatibility)
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((pi_ip, pi_port))
        
        message = command.encode('utf-8')
        sock.sendall(message)
        
        response = sock.recv(1024).decode('utf-8')
        sock.close()
        
        return {
            'success': True,
            'response': response,
            'timestamp': time.time()
        }
        
    except Exception as e:
        return {
            'success': False,
            'error': str(e),
            'timestamp': time.time()
        }

def check_connection(pi_ip, pi_port, timeout=3):
    """
    Check if the Raspberry Pi server is reachable (legacy function)
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        
        result = sock.connect_ex((pi_ip, pi_port))
        sock.close()
        
        if result == 0:
            return {
                'connected': True,
                'status': 'online',
                'timestamp': time.time()
            }
        else:
            return {
                'connected': False,
                'status': 'offline',
                'timestamp': time.time()
            }
            
    except Exception as e:
        return {
            'connected': False,
            'status': 'error',
            'error': str(e),
            'timestamp': time.time()
        }

# Test function
if __name__ == "__main__":
    # Test the long-lived socket client
    test_ip = "192.168.16.154"
    test_port = 8888
    
    print("🔌 Testing long-lived socket connection...")
    
    # Create client
    client = RobotSocketClient(test_ip, test_port)
    
    # Set up callbacks
    def on_response(command, response, success):
        print(f"📥 Response to '{command}': {response}")
    
    def on_connection(connected, message):
        print(f"🔗 Connection status: {connected} - {message}")
    
    client.set_response_callback(on_response)
    client.set_connection_callback(on_connection)
    
    # Connect
    if client.connect():
        try:
            # Test single commands
            print("\n📤 Testing single commands...")
            client.send_command("MODE:manual")
            time.sleep(1)
            
            # Test continuous commands
            print("\n🔄 Testing continuous commands (3 seconds)...")
            client.start_continuous_command("MOVE:forward", 0.2)
            time.sleep(3)
            client.stop_continuous_command()
            
            # Stop robot
            client.send_command("MOVE:stop")
            time.sleep(1)
            
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted")
        
        finally:
            client.disconnect()
    else:
        print("❌ Cannot connect to Pi server. Make sure server_ros2.py is running on the Pi.")