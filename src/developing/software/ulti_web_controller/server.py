#!/usr/bin/env python3
"""
ROS2 Robot Controller Server - Modified Version
Removed gyro mode, updated follow mode with distance-based control
Supports manual and follow modes with video streaming
"""

import socket
import threading
import time
import json
from datetime import datetime
import select
import cv2
import numpy as np
import serial
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

# Configuration
HOST = '0.0.0.0'
PORT = 8888
VIDEO_PORT = 8889
MAX_CONNECTIONS = 5
COMMAND_TIMEOUT = 0.5

# Serial configuration for motor controller
MOTOR_SERIAL_PORT = '/dev/ttyUSB0'  # Adjust this to your ESP32 serial port
MOTOR_BAUD_RATE = 115200

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize robot state
        self.current_mode = "manual"
        self.is_moving = False
        self.last_command = None
        self.command_count = 0
        self.start_time = time.time()
        self.last_move_command_time = 0
        
        # Distance data from ultrasonic sensor
        self.ultrasonic_distance = 999.0  # cm
        self.last_distance_time = 0
        self.min_follow_distance = 30.0  # Don't move forward if closer than 30cm
        self.safe_follow_distance = 50.0  # Ideal following distance
        
        # Serial connection to motor controller
        self.motor_serial = None
        self.init_motor_serial()
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        
        # Setup QoS profile to match camera publisher
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers with matching QoS
        self.image_subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, image_qos_profile
        )
        
        # Subscribe to tracking data for follow mode - expecting Float64
        self.tracker_subscriber = self.create_subscription(
            Float64, '/tracker_data', self.tracker_callback, 10
        )
        
        # Movement parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s
        
        # Timer for publishing status and checking command timeout
        self.status_timer = self.create_timer(0.1, self.update_loop)
        
        # Timer for reading serial data from motor controller
        self.serial_timer = self.create_timer(0.05, self.read_motor_serial)
        
        # Current movement state
        self.current_twist = Twist()
        
        # Video streaming
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # Tracking data for follow mode
        self.tracker_error = 0.0
        self.tracker_valid = False
        self.last_tracker_time = 0
        self.tracker_timeout = 1.0  # seconds
        
        # Mode-specific variables
        self.follow_state = 'searching'
        
        # Follow mode parameters
        self.follow_kp = 0.003  # Proportional gain for turning
        self.follow_error_threshold = 30  # pixels - dead zone for centering
        self.follow_max_error = 320  # half of 640 width
        
        self.get_logger().info('ROS2 Robot Controller initialized')
        self.get_logger().info('Modes: manual, follow')
        self.get_logger().info('Video streaming enabled on port 8889')
        self.get_logger().info('Follow mode with distance-based control')
    
    def init_motor_serial(self):
        """Initialize serial connection to motor controller"""
        try:
            self.motor_serial = serial.Serial(
                port=MOTOR_SERIAL_PORT,
                baudrate=MOTOR_BAUD_RATE,
                timeout=0.1
            )
            self.get_logger().info(f'Motor serial connected on {MOTOR_SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'Failed to open motor serial port: {e}')
            self.get_logger().warning('Continuing without motor serial connection')
            self.motor_serial = None
    
    def read_motor_serial(self):
        """Read serial data from motor controller"""
        if not self.motor_serial:
            return
        
        try:
            if self.motor_serial.in_waiting > 0:
                line = self.motor_serial.readline().decode('utf-8').strip()
                
                # Parse distance data: "DIST:XXX.XX,timestamp"
                if line.startswith("DIST:"):
                    match = re.match(r"DIST:([\d.]+),(\d+)", line)
                    if match:
                        distance = float(match.group(1))
                        timestamp = int(match.group(2))
                        
                        self.ultrasonic_distance = distance
                        self.last_distance_time = time.time()
                        
                        # Log distance periodically
                        if hasattr(self, '_dist_log_count'):
                            self._dist_log_count += 1
                        else:
                            self._dist_log_count = 1
                        
                        if self._dist_log_count % 20 == 0:  # Log every 20th reading
                            self.get_logger().info(f'Distance: {distance:.1f}cm')
                
                # Handle emergency stop notifications
                elif line.startswith("ESTOP:"):
                    self.get_logger().warning(f'Motor controller: {line}')
                    
        except Exception as e:
            if "device reports readiness" not in str(e):  # Ignore common non-critical errors
                self.get_logger().error(f'Error reading motor serial: {e}')
    
    def send_motor_command(self, vel1, vel2, vel3):
        """Send velocity command to motor controller"""
        if not self.motor_serial:
            return False
        
        try:
            # Format: "vel1,vel2,vel3\n"
            command = f"{vel1:.1f},{vel2:.1f},{vel3:.1f}\n"
            self.motor_serial.write(command.encode('utf-8'))
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending motor command: {e}')
            return False
    
    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            # Convert ROS Image to CV2 format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.image_lock:
                self.latest_image = cv_image
                
            # Log successful image reception (every 150 frames = ~5 seconds at 30fps)
            if hasattr(self, '_image_count'):
                self._image_count += 1
            else:
                self._image_count = 1
                
            if self._image_count % 150 == 0:
                height, width = cv_image.shape[:2]
                self.get_logger().info(f'Receiving images: {width}x{height}, count: {self._image_count}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def tracker_callback(self, msg):
        """Handle object tracking data - receives Float64 error value"""
        try:
            # Get error value from Float64 message
            self.tracker_error = msg.data
            self.last_tracker_time = time.time()
            
            # Consider tracking valid if error is not exactly 0 (0 means no detection in YOLO.py)
            self.tracker_valid = (self.tracker_error != 0.0)
            
            # Log tracking data periodically
            if hasattr(self, '_tracker_count'):
                self._tracker_count += 1
            else:
                self._tracker_count = 1
            
            if self._tracker_count % 10 == 0:  # Log every 10th message
                self.get_logger().info(f'Tracker - Error: {self.tracker_error:.2f}, Valid: {self.tracker_valid}')
            
            # Run follow mode logic if active
            if self.current_mode == 'follow':
                self.follow_object()
                
        except Exception as e:
            self.get_logger().error(f'Error processing tracker data: {e}')
    
    def update_loop(self):
        """Main update loop"""
        current_time = time.time()
        
        # Check for tracker timeout in follow mode
        if self.current_mode == 'follow':
            if current_time - self.last_tracker_time > self.tracker_timeout:
                self.tracker_valid = False
                self.follow_state = 'searching'
                # Stop the robot if we lose tracking
                if self.is_moving:
                    self.stop_robot()
        
        # Check for command timeout in manual mode
        if self.current_mode == 'manual' and self.is_moving:
            if current_time - self.last_move_command_time > COMMAND_TIMEOUT:
                self.get_logger().info('Movement timeout - stopping robot')
                self.stop_robot()
        
        # Always publish current twist
        self.cmd_vel_publisher.publish(self.current_twist)
        
        # Convert twist to motor commands for omnidirectional wheels
        # This is a simplified conversion - adjust based on your wheel configuration
        if self.is_moving:
            # For omnidirectional movement with 3 wheels at 120 degrees
            # This is a basic implementation - adjust for your specific robot
            linear_x = self.current_twist.linear.x * 1000  # Convert m/s to mm/s
            angular_z = self.current_twist.angular.z * 100  # Scale angular velocity
            
            # Simple differential drive for now (adjust for your omnidirectional setup)
            vel1 = linear_x - angular_z
            vel2 = linear_x + angular_z
            vel3 = linear_x
            
            self.send_motor_command(vel1, vel2, vel3)
        else:
            self.send_motor_command(0, 0, 0)
        
        # Publish status less frequently
        if int(current_time / 2) != int((current_time - 0.1) / 2):
            self.publish_status()
    
    def publish_status(self):
        """Publish robot status"""
        status = self.get_status()
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)
    
    def handle_mode_command(self, mode):
        """Handle mode switching"""
        valid_modes = ['manual', 'follow']  # Removed 'gyro'
        
        if mode in valid_modes:
            old_mode = self.current_mode
            self.current_mode = mode
            
            self.get_logger().info(f'Mode changed: {old_mode} → {mode}')
            
            # Publish mode change
            mode_msg = String()
            mode_msg.data = mode
            self.mode_publisher.publish(mode_msg)
            
            # Stop movement when switching modes
            if self.is_moving:
                self.stop_robot()
            
            # Mode-specific initialization
            if mode == 'follow':
                self.start_follow_mode()
            
            return f"MODE_OK:{mode}"
        else:
            return f"MODE_ERROR:Invalid mode {mode}"
    
    def handle_move_command(self, direction):
        """Handle movement commands for manual mode"""
        if self.current_mode != 'manual':
            return f"MOVE_ERROR:Not in manual mode (current: {self.current_mode})"
        
        return self._execute_move_command(direction)
    
    def _execute_move_command(self, direction):
        """Execute movement command"""
        valid_directions = ['forward', 'backward', 'left', 'right', 'stop']
        
        if direction not in valid_directions:
            return f"MOVE_ERROR:Invalid direction {direction}"
        
        # Update last command time
        self.last_move_command_time = time.time()
        
        # Update current twist
        if direction == 'forward':
            self.current_twist.linear.x = self.linear_speed
            self.current_twist.angular.z = 0.0
            self.is_moving = True
        elif direction == 'backward':
            self.current_twist.linear.x = -self.linear_speed
            self.current_twist.angular.z = 0.0
            self.is_moving = True
        elif direction == 'left':
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = self.angular_speed
            self.is_moving = True
        elif direction == 'right':
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = -self.angular_speed
            self.is_moving = True
        elif direction == 'stop':
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0
            self.is_moving = False
        
        return f"MOVE_OK:{direction}"
    
    def stop_robot(self):
        """Stop all robot movement"""
        self.is_moving = False
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.angular.z = 0.0
        
        # Send stop command to motors
        self.send_motor_command(0, 0, 0)
        
        # Publish stop command immediately
        self.cmd_vel_publisher.publish(self.current_twist)
    
    def start_follow_mode(self):
        """Initialize follow mode"""
        self.get_logger().info('Starting object following mode with distance control')
        self.get_logger().info(f'Follow parameters: Kp={self.follow_kp}, threshold={self.follow_error_threshold}px')
        self.get_logger().info(f'Distance limits: min={self.min_follow_distance}cm, safe={self.safe_follow_distance}cm')
        self.follow_state = 'searching'
        self.tracker_valid = False
    
    def follow_object(self):
        """Follow tracked object with distance-based speed control"""
        if self.current_mode != 'follow':
            return
        
        if not self.tracker_valid:
            # No valid tracking - stop
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0
            self.follow_state = 'searching'
            self.is_moving = False
            return
        
        # We have valid tracking data
        error = self.tracker_error  # Error in pixels from center
        distance = self.ultrasonic_distance  # Distance in cm
        
        # Calculate turning speed using proportional control
        # Negative error means object is to the left, positive means to the right
        turn_speed = -self.follow_kp * error  # Negative because we want to turn toward the object
        
        # Limit turn speed
        max_turn_speed = self.angular_speed * 0.7  # Use 70% of max angular speed
        turn_speed = max(-max_turn_speed, min(max_turn_speed, turn_speed))
        
        # Determine forward speed based on centering error AND distance
        forward_speed = 0.0
        
        # Check if object is centered enough
        if abs(error) < self.follow_error_threshold:
            # Object is centered - check distance to determine forward speed
            if distance > self.safe_follow_distance:
                # Too far - move forward at normal speed
                forward_speed = self.linear_speed
                self.follow_state = 'approaching'
            elif distance > self.min_follow_distance:
                # In safe zone - move forward slowly
                # Scale speed based on distance (closer = slower)
                speed_factor = (distance - self.min_follow_distance) / (self.safe_follow_distance - self.min_follow_distance)
                forward_speed = self.linear_speed * speed_factor * 0.5  # Max 50% speed in this zone
                self.follow_state = 'following'
            else:
                # Too close - stop or back up slightly
                if distance < 20:  # Very close to emergency stop distance
                    forward_speed = -self.linear_speed * 0.3  # Back up slowly
                    self.follow_state = 'too_close'
                else:
                    forward_speed = 0.0  # Just stop
                    self.follow_state = 'maintaining'
            
            # Apply small correction while moving
            self.current_twist.linear.x = forward_speed
            self.current_twist.angular.z = turn_speed * 0.2  # Small correction
            
        elif abs(error) < self.follow_error_threshold * 3:
            # Object somewhat centered - check distance before moving
            if distance > self.min_follow_distance:
                # Safe to move slowly while turning
                forward_speed = self.linear_speed * 0.3
            else:
                forward_speed = 0.0
            
            self.current_twist.linear.x = forward_speed
            self.current_twist.angular.z = turn_speed
            self.follow_state = 'centering'
            
        else:
            # Object far from center - just turn, no forward movement
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = turn_speed
            self.follow_state = 'turning'
        
        self.is_moving = (abs(self.current_twist.linear.x) > 0.01 or abs(self.current_twist.angular.z) > 0.01)
        
        # Log follow state periodically
        if hasattr(self, '_follow_log_count'):
            self._follow_log_count += 1
        else:
            self._follow_log_count = 1
        
        if self._follow_log_count % 20 == 0:  # Log every 20th call
            self.get_logger().info(
                f'Follow - State: {self.follow_state}, Error: {error:.1f}px, Distance: {distance:.1f}cm, '
                f'Linear: {self.current_twist.linear.x:.2f}, Angular: {self.current_twist.angular.z:.2f}'
            )
    
    def get_status(self):
        """Get current robot status"""
        uptime = time.time() - self.start_time
        
        status = {
            'mode': self.current_mode,
            'is_moving': self.is_moving,
            'last_command': self.last_command,
            'command_count': self.command_count,
            'uptime': round(uptime, 1),
            'linear_speed': self.current_twist.linear.x,
            'angular_speed': self.current_twist.angular.z,
            'timestamp': datetime.now().isoformat(),
            'image_count': getattr(self, '_image_count', 0),
            'ultrasonic_distance': round(self.ultrasonic_distance, 1)
        }
        
        # Add mode-specific data
        if self.current_mode == 'follow':
            status['follow_state'] = self.follow_state
            status['tracker_error'] = self.tracker_error
            status['tracker_valid'] = self.tracker_valid
        
        return status
    
    def process_command(self, command_str):
        """Process incoming command"""
        try:
            command_str = command_str.strip()
            self.command_count += 1
            
            # Track last non-movement command separately
            if not command_str.startswith('MOVE:'):
                self.last_command = command_str
                self.get_logger().info(f'Command #{self.command_count}: {command_str}')
            else:
                self.last_command = command_str
            
            if ':' in command_str:
                command_type, command_value = command_str.split(':', 1)
                
                if command_type == 'MODE':
                    return self.handle_mode_command(command_value)
                elif command_type == 'MOVE':
                    return self.handle_move_command(command_value)
                elif command_type == 'GET_TRACKING':
                    track_str = f"status={self.follow_state},"
                    track_str += f"error={int(self.tracker_error)},"
                    track_str += f"distance={int(self.ultrasonic_distance)},"
                    track_str += f"action={'following' if self.tracker_valid else 'searching'}"
                    return f"TRACKING_DATA:{track_str}"
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
            self.get_logger().error(error_msg)
            return error_msg

class VideoStreamServer(threading.Thread):
    """Separate server for video streaming"""
    def __init__(self, robot_node, port):
        super().__init__(daemon=True)
        self.robot_node = robot_node
        self.port = port
        self.running = False
        self.server_socket = None
        self.clients = []
        self.client_lock = threading.Lock()
    
    def run(self):
        """Run video streaming server"""
        self.running = True
        
        # Create server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.server_socket.bind((HOST, self.port))
            self.server_socket.listen(5)
            
            self.robot_node.get_logger().info(f'Video server listening on port {self.port}')
            
            # Accept clients in a separate thread
            accept_thread = threading.Thread(target=self.accept_clients, daemon=True)
            accept_thread.start()
            
            # Stream video frames
            while self.running:
                with self.robot_node.image_lock:
                    if self.robot_node.latest_image is not None:
                        # Resize image to 640x480 if needed
                        height, width = self.robot_node.latest_image.shape[:2]
                        if width != 640 or height != 480:
                            resized = cv2.resize(self.robot_node.latest_image, (640, 480))
                        else:
                            resized = self.robot_node.latest_image
                        
                        # Encode image as JPEG with quality setting
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                        _, buffer = cv2.imencode('.jpg', resized, encode_param)
                        frame_data = buffer.tobytes()
                        
                        # Send to all connected clients
                        with self.client_lock:
                            for client in self.clients[:]:
                                try:
                                    client.sendall(frame_data)
                                    client.sendall(b'\xff\xd9')  # JPEG end marker
                                except:
                                    self.clients.remove(client)
                                    client.close()
                
                time.sleep(0.033)  # ~30 FPS
                
        except Exception as e:
            self.robot_node.get_logger().error(f'Video server error: {e}')
        finally:
            self.cleanup()
    
    def accept_clients(self):
        """Accept video streaming clients"""
        while self.running:
            try:
                client, addr = self.server_socket.accept()
                with self.client_lock:
                    self.clients.append(client)
                self.robot_node.get_logger().info(f'Video client connected from {addr}')
            except:
                if self.running:
                    time.sleep(0.1)
    
    def cleanup(self):
        """Clean up video server"""
        self.running = False
        
        with self.client_lock:
            for client in self.clients:
                try:
                    client.close()
                except:
                    pass
            self.clients.clear()
        
        if self.server_socket:
            self.server_socket.close()

class RobotServer:
    def __init__(self, robot_node):
        self.robot_node = robot_node
        self.server_socket = None
        self.running = False
        self.active_clients = []
        self.client_lock = threading.Lock()
    
    def start(self):
        """Start the socket server"""
        self.robot_node.get_logger().info('Starting socket server...')
        
        # Create socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.server_socket.bind((HOST, PORT))
            self.server_socket.listen(MAX_CONNECTIONS)
            
            self.robot_node.get_logger().info(f'Server listening on {HOST}:{PORT}')
            
            self.running = True
            
            while self.running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    
                    with self.client_lock:
                        self.active_clients.append(client_socket)
                    
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, client_address),
                        daemon=True
                    )
                    client_thread.start()
                    
                except socket.error as e:
                    if self.running:
                        self.robot_node.get_logger().error(f'Socket error: {e}')
                    break
                    
        except Exception as e:
            self.robot_node.get_logger().error(f'Server error: {e}')
        finally:
            self.cleanup()
    
    def handle_client(self, client_socket, client_address):
        """Handle individual client connection"""
        self.robot_node.get_logger().info(f'New connection from {client_address}')
        
        client_socket.settimeout(0.1)
        
        try:
            while self.running:
                try:
                    ready, _, _ = select.select([client_socket], [], [], 0.1)
                    
                    if ready:
                        data = client_socket.recv(1024)
                        if not data:
                            break
                        
                        command = data.decode('utf-8').strip()
                        response = self.robot_node.process_command(command)
                        
                        try:
                            client_socket.send(response.encode('utf-8'))
                        except socket.error:
                            break
                    
                    time.sleep(0.01)
                    
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    break
                    
        except Exception as e:
            self.robot_node.get_logger().error(f'Client handler error: {e}')
        finally:
            with self.client_lock:
                if client_socket in self.active_clients:
                    self.active_clients.remove(client_socket)
            
            client_socket.close()
            self.robot_node.get_logger().info(f'Connection closed with {client_address}')
    
    def stop(self):
        """Stop the server"""
        self.running = False
        
        with self.client_lock:
            for client_socket in self.active_clients:
                try:
                    client_socket.close()
                except:
                    pass
            self.active_clients.clear()
        
        if self.server_socket:
            self.server_socket.close()
    
    def cleanup(self):
        """Cleanup resources"""
        self.robot_node.stop_robot()
        self.stop()

def main(args=None):
    """Main function - Entry point for ROS2"""
    print("🤖 ROS2 Robot Controller Starting...")
    print("📹 Video streaming enabled (640x480)")
    print("🎮 Modes: manual, follow")
    print("🎯 Follow mode with distance-based speed control")
    print("📏 Ultrasonic distance data from motor controller")
    print("=" * 50)
    
    rclpy.init(args=args)
    
    try:
        # Create robot node
        robot_node = RobotControllerNode()
        
        # Create servers
        server = RobotServer(robot_node)
        video_server = VideoStreamServer(robot_node, VIDEO_PORT)
        
        # Start servers in separate threads
        server_thread = threading.Thread(target=server.start, daemon=True)
        server_thread.start()
        
        video_server.start()
        
        robot_node.get_logger().info('All systems ready!')
        
        # Spin the node
        rclpy.spin(robot_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        print("🧹 Shutting down...")
        if 'server' in locals():
            server.stop()
        if 'video_server' in locals():
            video_server.running = False
        if 'robot_node' in locals():
            if robot_node.motor_serial:
                robot_node.motor_serial.close()
            robot_node.destroy_node()
        rclpy.shutdown()
        print("✅ Robot Controller stopped")

if __name__ == "__main__":
    main()