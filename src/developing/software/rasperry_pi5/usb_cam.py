#!/usr/bin/env python3
# coding=utf-8

"""
Fixed Camera Publisher Node for ROS 2
Captures webcam feed with better error handling
"""
#caution:camera resolution aligned with YOLO 
import rclpy
import cv2
import cv_bridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Parameters with defaults
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 1920)  
        self.declare_parameter('frame_height', 1080) 
        self.declare_parameter('fps', 30)
        self.declare_parameter('use_v4l2', True)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.use_v4l2 = self.get_parameter('use_v4l2').value
        
        # Initialize CV bridge
        self.bridge = cv_bridge.CvBridge()
        
        # Setup camera with multiple attempts
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
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.get_logger().info('Camera publisher started successfully')
        self.frame_count = 0
        self.error_count = 0
    
    def setup_camera(self):
        """Initialize and configure the camera with multiple backend attempts"""
        backends = []
        
        if self.use_v4l2:
            backends.append(('V4L2', cv2.CAP_V4L2))
        backends.extend([
            ('Default', cv2.CAP_ANY),
            ('V4L', cv2.CAP_V4L),
            ('GStreamer', cv2.CAP_GSTREAMER)
        ])
        
        # Try different backends
        for backend_name, backend in backends:
            self.get_logger().info(f'Trying camera index {self.camera_index} with {backend_name} backend...')
            
            if backend == cv2.CAP_ANY:
                self.cap = cv2.VideoCapture(self.camera_index)
            else:
                self.cap = cv2.VideoCapture(self.camera_index, backend)
            
            if self.cap.isOpened():
                self.get_logger().info(f'Successfully opened camera with {backend_name} backend')
                break
        
        if not self.cap.isOpened():
            # List available cameras for debugging
            self.get_logger().error(f'Failed to open camera at index {self.camera_index}')
            self.get_logger().info('Checking available cameras...')
            for i in range(5):
                test_cap = cv2.VideoCapture(i)
                if test_cap.isOpened():
                    self.get_logger().info(f'Camera found at index {i}')
                    test_cap.release()
            raise RuntimeError('Camera initialization failed')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency
        
        # Verify settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps')
        
        # Test capture
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error('Camera opened but cannot capture frames')
            raise RuntimeError('Camera capture test failed')
        self.get_logger().info('Camera capture test successful')
    
    def publish_frame(self):
        """Capture and publish a single frame"""
        ret, frame = self.cap.read()
        
        if not ret or frame is None:
            self.error_count += 1
            if self.error_count > 10:
                self.get_logger().error('Too many capture failures, attempting to reconnect...')
                try:
                    self.cap.release()
                    self.setup_camera()
                    self.error_count = 0
                except Exception as e:
                    self.get_logger().error(f'Reconnection failed: {str(e)}')
            else:
                self.get_logger().warn(f'Failed to capture frame (error count: {self.error_count})')
            return
        
        # Reset error count on successful capture
        self.error_count = 0
        
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
            if self.frame_count % (self.fps * 5) == 0:  # Log every 5 seconds
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources when shutting down"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released')
        super().destroy_node()

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = CameraPublisher()
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
