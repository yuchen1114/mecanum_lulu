#!/usr/bin/env python3
"""
ROS2 Robot Controller Server
This script runs on the Raspberry Pi and listens for commands from the web controller,
then publishes them to ROS2 topics.
"""

import socket
import threading
import time
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 8888
MAX_CONNECTIONS = 5

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_server')
        
        # Initialize robot state
        self.current_mode = "manual"
        self.is_moving = False
        self.last_command = None
        self.command_count = 0
        self.start_time = time.time()
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        
        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        
        # Timer for publishing status
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('ROS2 Robot Controller Node initialized')
        self.get_logger().info('Waiting for commands from web server at localhost:5000')
        
    def publish_status(self):
        """Publish robot status periodically"""
        status = self.get_status()
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)
    
    def handle_mode_command(self, mode):
        """Handle mode switching commands"""
        valid_modes = ['manual', 'auto', 'follow']
        
        if mode in valid_modes:
            old_mode = self.current_mode
            self.current_mode = mode
            
            self.get_logger().info(f'Mode changed: {old_mode} → {mode}')
            
            # Publish mode change
            mode_msg = String()
            mode_msg.data = mode
            self.mode_publisher.publish(mode_msg)
            
            # Stop any current movement when switching modes
            if self.is_moving:
                self.stop_robot()
            
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
        
        self.get_logger().info(f'Moving: {direction}')
        
        # Create Twist message
        twist_msg = Twist()
        
        # Execute movement
        if direction == 'forward':
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = 0.0
            self.is_moving = True
        elif direction == 'backward':
            twist_msg.linear.x = -self.linear_speed
            twist_msg.angular.z = 0.0
            self.is_moving = True
        elif direction == 'left':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_speed
            self.is_moving = True
        elif direction == 'right':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -self.angular_speed
            self.is_moving = True
        elif direction == 'stop':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.is_moving = False
        
        # Publish movement command
        self.cmd_vel_publisher.publish(twist_msg)
        
        return f"MOVE_OK:{direction}"
    
    def stop_robot(self):
        """Stop all robot movement"""
        self.is_moving = False
        self.get_logger().info('Stopping robot')
        
        # Publish stop command
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
    
    def start_auto_mode(self):
        """Initialize autonomous mode"""
        self.get_logger().info('Starting autonomous mode')
        # This mode can be handled by other ROS2 nodes
        # Example: navigation stack, path planning nodes
        # The mode change is published to /robot_mode topic
    
    def start_follow_mode(self):
        """Initialize follow mode"""
        self.get_logger().info('Starting follow mode')
        # This mode can be handled by other ROS2 nodes
        # Example: person detection, object tracking nodes
        # The mode change is published to /robot_mode topic
    
    def stop_auto_behaviors(self):
        """Stop any autonomous behaviors"""
        self.get_logger().info('Stopping autonomous behaviors')
        self.stop_robot()
    
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
            
            self.get_logger().info(f'Command #{self.command_count}: {command_str}')
            
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
            self.get_logger().error(error_msg)
            return error_msg

class RobotServer:
    def __init__(self, robot_node):
        self.robot_node = robot_node
        self.server_socket = None
        self.running = False
    
    def start(self):
        """Start the socket server"""
        self.robot_node.get_logger().info('Starting socket server...')
        
        # Create socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            # Bind socket to address
            self.server_socket.bind((HOST, PORT))
            self.server_socket.listen(MAX_CONNECTIONS)
            
            self.robot_node.get_logger().info(f'Server listening on {HOST}:{PORT}')
            self.robot_node.get_logger().info(f'Max connections: {MAX_CONNECTIONS}')
            
            self.running = True
            
            while self.running:
                try:
                    # Accept incoming connection
                    client_socket, client_address = self.server_socket.accept()
                    
                    # Handle client in separate thread
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
                    self.robot_node.get_logger().error(f'Error accepting connection: {e}')
                    
        except Exception as e:
            self.robot_node.get_logger().error(f'Server error: {e}')
        finally:
            self.cleanup()
    
    def handle_client(self, client_socket, client_address):
        """Handle individual client connection"""
        self.robot_node.get_logger().info(f'New connection from {client_address}')
        
        try:
            while self.running:
                # Receive data from client
                data = client_socket.recv(1024)
                if not data:
                    break
                
                command = data.decode('utf-8')
                
                # Process command
                response = self.robot_node.process_command(command)
                
                # Send response back to client
                client_socket.send(response.encode('utf-8'))
                
                self.robot_node.get_logger().info(f'Response to {client_address}: {response}')
                
        except ConnectionResetError:
            self.robot_node.get_logger().info(f'Connection reset by {client_address}')
        except Exception as e:
            self.robot_node.get_logger().error(f'Error handling client {client_address}: {e}')
        finally:
            client_socket.close()
            self.robot_node.get_logger().info(f'Connection closed with {client_address}')
    
    def stop(self):
        """Stop the server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
    
    def cleanup(self):
        """Cleanup resources"""
        self.robot_node.get_logger().info('Cleaning up server...')
        self.robot_node.stop_robot()
        if self.server_socket:
            self.server_socket.close()

def main(args=None):
    """Main function"""
    print("🤖 ROS2 Robot Controller Server Starting...")
    print("=" * 50)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create robot node
        robot_node = RobotControllerNode()
        
        # Create server
        server = RobotServer(robot_node)
        
        # Start server in separate thread
        server_thread = threading.Thread(target=server.start, daemon=True)
        server_thread.start()
        
        robot_node.get_logger().info('ROS2 Robot Controller ready!')
        robot_node.get_logger().info('Listening for web commands on port 8888')
        robot_node.get_logger().info('Publishing to ROS2 topics:')
        robot_node.get_logger().info('  - /cmd_vel (geometry_msgs/Twist)')
        robot_node.get_logger().info('  - /robot_mode (std_msgs/String)')
        robot_node.get_logger().info('  - /robot_status (std_msgs/String)')
        robot_node.get_logger().info('Web interface available at: http://localhost:5000')
        
        # Spin the node
        rclpy.spin(robot_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Server shutdown requested")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Cleanup
        print("🧹 Shutting down...")
        if 'server' in locals():
            server.stop()
        if 'robot_node' in locals():
            robot_node.destroy_node()
        rclpy.shutdown()
        print("✅ ROS2 Robot Controller stopped")

if __name__ == "__main__":
    main()