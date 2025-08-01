#!/usr/bin/env python3
"""
ROS2 Robot Controller Server
Supports manual, follow, and gyro modes with video streaming
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
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

# Configuration
HOST = '0.0.0.0'
PORT = 8888
VIDEO_PORT = 8889
MAX_CONNECTIONS = 5
COMMAND_TIMEOUT = 0.5

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
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        
        # Create subscribers
        # Subscribe to raw image - we'll compress it ourselves for better control
        self.image_subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
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
        
        # Tracking data for follow mode
        self.tracker_error = 0
        self.tracker_valid = False
        
        # Mode-specific variables
        self.follow_state = 'searching'
        self.gyro_pitch = 0
        self.gyro_roll = 0
        
        self.get_logger().info('ROS2 Robot Controller initialized')
        self.get_logger().info('Modes: manual, follow, gyro')
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
        valid_modes = ['manual', 'follow', 'gyro']
        
        if mode in valid_modes:
            old_mode = self.current_mode
            self.current_mode = mode
            
            self.get_logger