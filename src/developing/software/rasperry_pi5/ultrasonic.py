#!/usr/bin/env python3
"""
Ultrasonic Sensor ROS2 Node for Raspberry Pi 5
Reads data from 3 HC-SR04 ultrasonic sensors and publishes to ROS2
Sensors are arranged 120 degrees apart for full coverage
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Header
import RPi.GPIO as GPIO
import time
import threading
import numpy as np

# GPIO Pin Configuration
SENSOR_CONFIG = {
    'left': {
        'trig': 23,
        'echo': 24,
        'frame_id': 'ultrasonic_left',
        'field_of_view': 0.26,  # ~15 degrees in radians
        'angle': 120  # degrees from front
    },
    'center': {
        'trig': 17,
        'echo': 27,
        'frame_id': 'ultrasonic_center',
        'field_of_view': 0.26,
        'angle': 0  # degrees from front
    },
    'right': {
        'trig': 5,
        'echo': 6,
        'frame_id': 'ultrasonic_right',
        'field_of_view': 0.26,
        'angle': -120  # degrees from front
    }
}

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Set up GPIO pins
        for sensor_name, config in SENSOR_CONFIG.items():
            GPIO.setup(config['trig'], GPIO.OUT)
            GPIO.setup(config['echo'], GPIO.IN)
            GPIO.output(config['trig'], GPIO.LOW)
        
        # Publishers for individual sensors
        self.range_publishers = {}
        for sensor_name in SENSOR_CONFIG.keys():
            topic_name = f'/ultrasonic_{sensor_name}'
            self.range_publishers[sensor_name] = self.create_publisher(
                Range, topic_name, 10
            )
        
        # Publisher for combined sensor data
        self.combined_publisher = self.create_publisher(
            Float32MultiArray, '/ultrasonic_array', 10
        )
        
        # Sensor parameters
        self.min_range = 0.02  # 2cm
        self.max_range = 4.0   # 400cm
        self.radiation_type = Range.ULTRASOUND
        
        # Data storage
        self.distances = {'left': 0.0, 'center': 0.0, 'right': 0.0}
        self.lock = threading.Lock()
        
        # Create timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        
        # Start measurement threads
        self.running = True
        self.measurement_threads = []
        for sensor_name in SENSOR_CONFIG.keys():
            thread = threading.Thread(
                target=self.measurement_loop,
                args=(sensor_name,),
                daemon=True
            )
            thread.start()
            self.measurement_threads.append(thread)
        
        self.get_logger().info('Ultrasonic sensor node initialized')
        self.get_logger().info(f'Publishing to topics:')
        for sensor_name in SENSOR_CONFIG.keys():
            self.get_logger().info(f'  - /ultrasonic_{sensor_name}')
        self.get_logger().info('  - /ultrasonic_array')
    
    def measurement_loop(self, sensor_name):
        """Continuously measure distance for a specific sensor"""
        config = SENSOR_CONFIG[sensor_name]
        
        while self.running:
            try:
                # Measure distance
                distance = self.measure_distance(config['trig'], config['echo'])
                
                # Update stored value
                with self.lock:
                    self.distances[sensor_name] = distance
                
                # Small delay between measurements
                time.sleep(0.05)
                
            except Exception as e:
                self.get_logger().error(f'Error in {sensor_name} sensor: {e}')
                time.sleep(0.1)
    
    def measure_distance(self, trig_pin, echo_pin):
        """Measure distance using HC-SR04 ultrasonic sensor"""
        # Send trigger pulse
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(trig_pin, GPIO.LOW)
        
        # Wait for echo to start
        pulse_start = time.time()
        timeout_start = pulse_start
        
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > 0.1:  # 100ms timeout
                return self.max_range
        
        # Wait for echo to end
        pulse_end = time.time()
        timeout_start = pulse_end
        
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > 0.1:  # 100ms timeout
                return self.max_range
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2
        distance = distance / 100  # Convert to meters
        
        # Clamp to valid range
        distance = max(self.min_range, min(distance, self.max_range))
        
        return distance
    
    def publish_sensor_data(self):
        """Publish sensor data to ROS2 topics"""
        current_time = self.get_clock().now()
        
        # Get current distances
        with self.lock:
            current_distances = self.distances.copy()
        
        # Publish individual Range messages
        for sensor_name, distance in current_distances.items():
            config = SENSOR_CONFIG[sensor_name]
            
            # Create Range message
            range_msg = Range()
            range_msg.header.stamp = current_time.to_msg()
            range_msg.header.frame_id = config['frame_id']
            range_msg.radiation_type = self.radiation_type
            range_msg.field_of_view = config['field_of_view']
            range_msg.min_range = self.min_range
            range_msg.max_range = self.max_range
            range_msg.range = float(distance)
            
            # Publish
            self.range_publishers[sensor_name].publish(range_msg)
        
        # Publish combined array
        array_msg = Float32MultiArray()
        array_msg.data = [
            float(current_distances['left']),
            float(current_distances['center']),
            float(current_distances['right'])
        ]
        self.combined_publisher.publish(array_msg)
        
        # Log occasionally
        if int(current_time.nanoseconds / 1e9) % 5 == 0:
            self.get_logger().info(
                f'Distances - L: {current_distances["left"]:.2f}m, '
                f'C: {current_distances["center"]:.2f}m, '
                f'R: {current_distances["right"]:.2f}m'
            )
    
    def get_sensor_data(self):
        """Get current sensor readings"""
        with self.lock:
            return self.distances.copy()
    
    def cleanup(self):
        """Clean up GPIO and stop threads"""
        self.running = False
        
        # Wait for threads to stop
        for thread in self.measurement_threads:
            thread.join(timeout=1.0)
        
        # Clean up GPIO
        GPIO.cleanup()
        
        self.get_logger().info('Ultrasonic sensor node cleaned up')

def main(args=None):
    """Main function"""
    print("🔊 Ultrasonic Sensor ROS2 Node Starting...")
    print("📡 3 HC-SR04 sensors configured at 120° intervals")
    print("=" * 50)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and run node
        ultrasonic_node = UltrasonicSensorNode()
        
        print("✅ Ultrasonic sensors initialized")
        print("📊 Publishing sensor data to ROS2 topics")
        
        # Spin the node
        rclpy.spin(ultrasonic_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Cleanup
        print("🧹 Cleaning up...")
        if 'ultrasonic_node' in locals():
            ultrasonic_node.cleanup()
            ultrasonic_node.destroy_node()
        rclpy.shutdown()
        print("✅ Ultrasonic sensor node stopped")

if __name__ == "__main__":
    main()