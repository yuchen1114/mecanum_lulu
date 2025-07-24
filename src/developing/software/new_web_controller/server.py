#!/usr/bin/env python3
"""
Enhanced ROS2 Robot Controller Server
Supports manual, auto, follow, and gyro modes with video streaming
"""

import socket
import threading
import time
import json
from datetime import datetime
import select
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage, Range
from cv_bridge import CvBridge

# Configuration
HOST = '0.0.0.0'
PORT = 8888
VIDEO_PORT = 8889
MAX_CONNECTIONS = 5
COMMAND_TIMEOUT = 0.5

class EnhancedRobotControllerNode(Node):
    def __init__(self):
        super().__init__('enhanced_robot_controller')
        
        # Initialize robot state
        self.current_mode = "manual"
        self.is_moving = False
        self.last_command = None
        self.command_count = 0
        self.start_time = time.time()
        self.last_move_command_time = 0
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        
        # Create subscribers
        # Subscribe to raw image - we'll compress it ourselves for better control
        self.image_subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        
        # Subscribe to ultrasonic sensors
        self.ultrasonic_subscriber = self.create_subscription(
            Float32MultiArray, '/ultrasonic_array', self.ultrasonic_callback, 10
        )
        
        # Subscribe to tracking data for follow mode
        self.tracker_subscriber = self.create_subscription(
            String, '/tracker_data', self.tracker_callback, 10
        )
        
        # Movement parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s
        
        # Timer for publishing status and checking command timeout
        self.status_timer = self.create_timer(0.1, self.update_loop)
        
        # Current movement state
        self.current_twist = Twist()
        
        # Video streaming
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # Sensor data
        self.ultrasonic_data = {'left': 0.0, 'center': 0.0, 'right': 0.0}
        self.tracker_error = 0
        self.tracker_valid = False
        
        # Mode-specific variables
        self.auto_state = 'exploring'
        self.follow_state = 'searching'
        self.gyro_pitch = 0
        self.gyro_roll = 0
        
        self.get_logger().info('Enhanced ROS2 Robot Controller initialized')
        self.get_logger().info('Modes: manual, auto, follow, gyro')
        self.get_logger().info('Video streaming enabled on port 8889')
    
    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            # Convert ROS Image to CV2 format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.image_lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def ultrasonic_callback(self, msg):
        """Handle ultrasonic sensor data"""
        if len(msg.data) >= 3:
            self.ultrasonic_data['left'] = msg.data[0]
            self.ultrasonic_data['center'] = msg.data[1]
            self.ultrasonic_data['right'] = msg.data[2]
            
            # Run auto mode logic if active
            if self.current_mode == 'auto':
                self.auto_navigate()
    
    def tracker_callback(self, msg):
        """Handle object tracking data"""
        try:
            data = json.loads(msg.data)
            self.tracker_error = data.get('error', 0)
            self.tracker_valid = data.get('valid', False)
            
            # Run follow mode logic if active
            if self.current_mode == 'follow':
                self.follow_object()
        except Exception as e:
            self.get_logger().error(f'Error processing tracker data: {e}')
    
    def update_loop(self):
        """Main update loop"""
        current_time = time.time()
        
        # Check for command timeout in manual/gyro mode
        if self.current_mode in ['manual', 'gyro'] and self.is_moving:
            if current_time - self.last_move_command_time > COMMAND_TIMEOUT:
                self.get_logger().info('Movement command timeout - stopping robot')
                self.stop_robot()
        
        # Always publish current twist
        self.cmd_vel_publisher.publish(self.current_twist)
        
        # Publish status periodically
        if int(current_time) != int(current_time - 0.1):
            self.publish_status()
    
    def publish_status(self):
        """Publish robot status"""
        status = self.get_status()
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)
    
    def handle_mode_command(self, mode):
        """Handle mode switching"""
        valid_modes = ['manual', 'auto', 'follow', 'gyro']
        
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
            if mode == 'auto':
                self.start_auto_mode()
            elif mode == 'follow':
                self.start_follow_mode()
            elif mode == 'gyro':
                self.start_gyro_mode()
            elif mode == 'manual':
                self.stop_auto_behaviors()
            
            return f"MODE_OK:{mode}"
        else:
            return f"MODE_ERROR:Invalid mode {mode}"
    
    def handle_move_command(self, direction):
        """Handle movement commands for manual mode"""
        if self.current_mode != 'manual':
            return f"MOVE_ERROR:Not in manual mode (current: {self.current_mode})"
        
        return self._execute_move_command(direction)
    
    def handle_gyro_command(self, command_str):
        """Handle gyro control commands"""
        if self.current_mode != 'gyro':
            return f"GYRO_ERROR:Not in gyro mode"
        
        try:
            # Parse gyro data: "pitch=XX,roll=XX,cmd=XX"
            parts = command_str.split(',')
            for part in parts:
                if '=' in part:
                    key, value = part.split('=')
                    if key == 'pitch':
                        self.gyro_pitch = float(value)
                    elif key == 'roll':
                        self.gyro_roll = float(value)
                    elif key == 'cmd':
                        # Execute movement based on gyro command
                        self._execute_move_command(value)
            
            return f"GYRO_OK"
        except Exception as e:
            return f"GYRO_ERROR:{str(e)}"
    
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
        self.cmd_vel_publisher.publish(self.current_twist)
    
    def start_auto_mode(self):
        """Initialize autonomous navigation mode"""
        self.get_logger().info('Starting autonomous navigation mode')
        self.auto_state = 'exploring'
    
    def start_follow_mode(self):
        """Initialize follow mode"""
        self.get_logger().info('Starting object following mode')
        self.follow_state = 'searching'
    
    def start_gyro_mode(self):
        """Initialize gyro control mode"""
        self.get_logger().info('Starting gyro control mode')
        self.gyro_pitch = 0
        self.gyro_roll = 0
    
    def stop_auto_behaviors(self):
        """Stop autonomous behaviors"""
        self.get_logger().info('Stopping autonomous behaviors')
        self.stop_robot()
    
    def auto_navigate(self):
        """Autonomous navigation using ultrasonic sensors"""
        if self.current_mode != 'auto':
            return
        
        # Get sensor readings
        left = self.ultrasonic_data['left']
        center = self.ultrasonic_data['center']
        right = self.ultrasonic_data['right']
        
        # Safety threshold (meters)
        obstacle_threshold = 0.3
        clear_threshold = 0.8
        
        # Determine action based on sensor readings
        if center < obstacle_threshold:
            # Obstacle ahead - decide which way to turn
            if left > right:
                self.current_twist.linear.x = 0.0
                self.current_twist.angular.z = self.angular_speed
                self.auto_state = 'turning_left'
            else:
                self.current_twist.linear.x = 0.0
                self.current_twist.angular.z = -self.angular_speed
                self.auto_state = 'turning_right'
        
        elif left < obstacle_threshold:
            # Obstacle on left - turn right
            self.current_twist.linear.x = self.linear_speed * 0.5
            self.current_twist.angular.z = -self.angular_speed * 0.5
            self.auto_state = 'avoiding_left'
        
        elif right < obstacle_threshold:
            # Obstacle on right - turn left
            self.current_twist.linear.x = self.linear_speed * 0.5
            self.current_twist.angular.z = self.angular_speed * 0.5
            self.auto_state = 'avoiding_right'
        
        else:
            # Path clear - move forward
            if center > clear_threshold:
                self.current_twist.linear.x = self.linear_speed
            else:
                self.current_twist.linear.x = self.linear_speed * 0.7
            self.current_twist.angular.z = 0.0
            self.auto_state = 'exploring'
        
        self.is_moving = True
    
    def follow_object(self):
        """Follow tracked object"""
        if self.current_mode != 'follow':
            return
        
        if not self.tracker_valid:
            # No valid tracking - stop or search
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0
            self.follow_state = 'searching'
            self.is_moving = False
            return
        
        # Proportional control based on error from center
        error_threshold = 50  # pixels (increased for 1080p)
        max_error = 960  # half of 1920 width
        
        if abs(self.tracker_error) < error_threshold:
            # Object centered - move forward
            self.current_twist.linear.x = self.linear_speed
            self.current_twist.angular.z = 0.0
            self.follow_state = 'following'
        else:
            # Turn to center object
            turn_speed = (self.tracker_error / max_error) * self.angular_speed
            self.current_twist.linear.x = self.linear_speed * 0.3
            self.current_twist.angular.z = -turn_speed  # Negative because error is positive when object is to the right
            self.follow_state = 'centering'
        
        self.is_moving = True
    
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
            'timestamp': datetime.now().isoformat()
        }
        
        # Add mode-specific data
        if self.current_mode == 'auto':
            status['auto_state'] = self.auto_state
            status['sensors'] = self.ultrasonic_data
        elif self.current_mode == 'follow':
            status['follow_state'] = self.follow_state
            status['tracker_error'] = self.tracker_error
            status['tracker_valid'] = self.tracker_valid
        elif self.current_mode == 'gyro':
            status['gyro_pitch'] = self.gyro_pitch
            status['gyro_roll'] = self.gyro_roll
        
        return status
    
    def process_command(self, command_str):
        """Process incoming command"""
        try:
            command_str = command_str.strip()
            self.last_command = command_str
            self.command_count += 1
            
            if ':' in command_str:
                command_type, command_value = command_str.split(':', 1)
                
                if command_type == 'MODE':
                    return self.handle_mode_command(command_value)
                elif command_type == 'MOVE':
                    return self.handle_move_command(command_value)
                elif command_type == 'GYRO':
                    return self.handle_gyro_command(command_value)
                elif command_type == 'GET_SENSORS':
                    sensor_str = f"left={self.ultrasonic_data['left']:.2f},"
                    sensor_str += f"center={self.ultrasonic_data['center']:.2f},"
                    sensor_str += f"right={self.ultrasonic_data['right']:.2f},"
                    sensor_str += f"status={self.auto_state}"
                    return f"SENSOR_DATA:{sensor_str}"
                elif command_type == 'GET_TRACKING':
                    track_str = f"status={self.follow_state},"
                    track_str += f"error={self.tracker_error},"
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
                        # Resize image for streaming (1920x1080 -> 960x540)
                        resized = cv2.resize(self.robot_node.latest_image, (960, 540))
                        
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
    """Main function"""
    print("🤖 Enhanced ROS2 Robot Controller Starting...")
    print("📹 Video streaming enabled")
    print("🎮 Modes: manual, auto, follow, gyro")
    print("=" * 50)
    
    rclpy.init(args=args)
    
    try:
        # Create robot node
        robot_node = EnhancedRobotControllerNode()
        
        # Create servers
        server = RobotServer(robot_node)
        video_server = VideoStreamServer(robot_node, VIDEO_PORT)
        
        # Start servers
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
            robot_node.destroy_node()
        rclpy.shutdown()
        print("✅ Enhanced Robot Controller stopped")

if __name__ == "__main__":
    main()