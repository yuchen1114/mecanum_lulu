#!/usr/bin/env python3
# coding=utf-8

"""
Object Tracking Node for ROS 2
Simplified autonomous object tracking system
"""

import rclpy
import cv2
import cv_bridge
import numpy as np
from ultralytics import YOLO
from rclpy.qos import QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_msg.msg import Sensordata

# Configuration
TARGET_Y = 320
MODEL_PATH = "/home/mingyang/wheeltec_ros2/src/object_tracking/object_tracking/best.pt"
CONFIDENCE_THRESHOLD = 0.80

class ObjectTracker:
    def __init__(self):
        self.model = YOLO(MODEL_PATH)
        self.bridge = cv_bridge.CvBridge()
    
    def process_frame(self, results):
        """Process YOLO detection results and return object center"""
        cx = TARGET_Y
        cy = 0
        
        # Get original frame
        frame = results[0].orig_img.copy()
        height, width = frame.shape[:2]
        center_x = width // 2
        
        # Draw reference line
        cv2.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)
        
        # Process detections
        detections = results[0].boxes
        class_names = getattr(results[0], 'names', self.model.names)
        
        for box in detections:
            confidence = float(box.conf)
            if confidence < CONFIDENCE_THRESHOLD:
                continue
            
            # Get bounding box coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            # Get class info
            class_id = int(box.cls[0])
            label = f"{class_names[class_id]} {confidence:.2f}"
            
            # Draw annotations
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        
        # Display frame
        cv2.imshow("Object Tracker", frame)
        cv2.waitKey(1)
        
        return cx + width // 2

class TrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')
        
        # Initialize tracker
        self.tracker = ObjectTracker()
        
        # Setup QoS
        qos = QoSProfile(depth=10)
        
        # Create publisher and subscriber
        self.publisher = self.create_publisher(Sensordata, 'tracker_data', qos)
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos)
    
    def image_callback(self, msg):
        """Process incoming image and publish tracking data"""
        try:
            # Convert ROS image to OpenCV format
            image = self.tracker.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.tracker.model(image, verbose=False)
            
            # Calculate tracking error
            height, width = image.shape[:2]
            center_offset = self.tracker.process_frame(results)
            error = width // 2 - center_offset + TARGET_Y
            
            # Apply boundary limits
            if error < (-width // 2 + 25):
                error = 0
            
            # Publish tracking data
            sensor_msg = Sensordata()
            sensor_msg.data = float(error)
            self.publisher.publish(sensor_msg)
            
            self.get_logger().info(f'Tracking error: {error}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    """Main function to run the tracker node"""
    rclpy.init(args=args)
    
    try:
        node = TrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()