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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# Configuration
TARGET_Y = 320
MODEL_PATH = "/home/lulu/mecanum_lulu/log/YOLO_model/lip/best.pt"
# MODEL_PATH = "/home/mecanum_lulu/log/YOLO_model/lip/best.pt"
CONFIDENCE_THRESHOLD = 0.70

class ObjectTracker:
    def __init__(self):
        self.model = YOLO(MODEL_PATH)
        self.bridge = cv_bridge.CvBridge()
    
    def process_frame(self, results):
        """Process YOLO detection results and return object center"""
        cx = TARGET_Y
        cy = 0
        object_detected = False
        
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
            object_detected = True
            
            # Get class info
            class_id = int(box.cls[0])
            label = f"{class_names[class_id]} {confidence:.2f}"
            
            # Draw annotations
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            
            # Only track the first valid detection
            break
        
        # Display frame
        cv2.imshow("Object Tracker", frame)
        cv2.waitKey(1)
        
        return cx, object_detected

class TrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')
        
        # Initialize tracker
        self.tracker = ObjectTracker()
        
        # Setup QoS to match camera publisher
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publisher and subscriber - using Float64 for simple error value
        self.publisher = self.create_publisher(Float64, '/tracker_data', 10)
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, image_qos)
    
    def image_callback(self, msg):
        """Process incoming image and publish tracking data"""
        try:
            # Convert ROS image to OpenCV format
            image = self.tracker.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.tracker.model(image, verbose=False)
            
            # Calculate tracking error
            height, width = image.shape[:2]
            center_x, object_detected = self.tracker.process_frame(results)
            
            # Calculate error from center
            if object_detected:
                error = center_x - (width // 2)
                
                # Apply boundary limits
                if error < (-width // 2 + 25):
                    error = 0.0
            else:
                # No object detected - send 0 to indicate no tracking
                error = 0.0
            
            # Publish tracking error as Float64
            error_msg = Float64()
            error_msg.data = float(error)
            self.publisher.publish(error_msg)
            
            status = "DETECTED" if object_detected else "NO_OBJECT"
            self.get_logger().info(f'Tracking - Error: {error:.2f}, Status: {status}')
            
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