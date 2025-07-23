#!/usr/bin/env python3
# coding=utf-8

"""
Camera Publisher Node for ROS 2
Captures webcam feed from Raspberry Pi 5 and publishes to /image_raw topic
"""

import rclpy
import cv2
import cv_bridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# Camera configuration
CAMERA_INDEX = 0  # Default webcam (change if needed)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

'''command for different params when running
ros2 run your_package camera_publisher.py --ros-args \
  -p camera_index:=0 \
  -p frame_width:=640 \
  -p frame_height:=480 \
  -p fps:=30 \
  -p show_preview:=true
  '''
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Initialize CV bridge
        self.bridge = cv_bridge.CvBridge()
        
        # Setup camera
        self.setup_camera()
        
        # Setup QoS profile for image streaming
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publisher
        self.publisher = self.create_publisher(Image, '/image_raw', qos_profile)
        
        # Create timer for publishing frames
        timer_period = 1.0 / FPS  # seconds
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.get_logger().info('Camera publisher started')
        self.frame_count = 0
    
    def setup_camera(self):
        """Initialize and configure the camera"""
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at index {CAMERA_INDEX}')
            raise RuntimeError('Camera initialization failed')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        
        # Verify settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps')
    
    def publish_frame(self):
        """Capture and publish a single frame"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        try:
            # Create ROS image message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_frame'
            
            # Convert OpenCV image to ROS image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = header
            
            # Publish the image
            self.publisher.publish(img_msg)
            
            self.frame_count += 1
            if self.frame_count % (FPS * 5) == 0:  # Log every 5 seconds
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources when shutting down"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

class CameraPublisherAdvanced(Node):
    """Advanced version with additional features"""
    def __init__(self):
        super().__init__('camera_publisher_advanced')
        
        # Parameters (can be set via launch files or command line)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('show_preview', False)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.show_preview = self.get_parameter('show_preview').value
        
        # Initialize components
        self.bridge = cv_bridge.CvBridge()
        self.setup_camera()
        
        # Setup publisher with QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(Image, '/image_raw', qos_profile)
        
        # Create timer
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.get_logger().info('Advanced camera publisher started')
        self.frame_count = 0
    
    def setup_camera(self):
        """Initialize camera with error handling"""
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {self.camera_index}')
            raise RuntimeError('Camera not available')
        
        # Configure camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # For Raspberry Pi, set additional properties
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency
        
        # Log actual settings
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        f = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera ready: {w}x{h} @ {f:.1f}fps')
    
    def publish_frame(self):
        """Capture and publish frame with optional preview"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Frame capture failed')
            return
        
        try:
            # Optional preview window
            if self.show_preview:
                cv2.imshow('Camera Feed', frame)
                cv2.waitKey(1)
            
            # Create message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_frame'
            
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = header
            
            # Publish
            self.publisher.publish(img_msg)
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Publishing error: {str(e)}')
    
    def destroy_node(self):
        """Cleanup"""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        # Use basic or advanced version
        node = CameraPublisher()  # Change to CameraPublisherAdvanced() for more features
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()