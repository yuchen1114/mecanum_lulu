#!/usr/bin/env python3
"""
Ultrasonic Sensor ROS2 Node for Raspberry Pi 5
Uses lgpio for optimal Pi 5 compatibility with Ubuntu 24.04
Reads data from 3 HC-SR04 ultrasonic sensors and publishes to ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Header
import time
import threading
import random
import os

# Try to import lgpio with detailed error handling
try:
    import lgpio
    GPIO_AVAILABLE = True
    print("✅ lgpio imported successfully")
except ImportError as e:
    print(f"❌ lgpio not available: {e}")
    print("💡 Install with: sudo apt install python3-lgpio")
    print("💡 Or try: pip3 install lgpio --break-system-packages")
    GPIO_AVAILABLE = False
    
    # Mock lgpio implementation for testing
    class MockLGPIO:
        def __init__(self):
            self.chip = None
            self.claimed_pins = set()
            
        def gpiochip_open(self, chip_num):
            print(f"[MOCK] Opening GPIO chip {chip_num}")
            return chip_num
            
        def gpio_claim_output(self, chip, pin):
            print(f"[MOCK] Claiming pin {pin} as output on chip {chip}")
            self.claimed_pins.add(pin)
            return 0
            
        def gpio_claim_input(self, chip, pin):
            print(f"[MOCK] Claiming pin {pin} as input on chip {chip}")
            self.claimed_pins.add(pin)
            return 0
            
        def gpio_write(self, chip, pin, value):
            # print(f"[MOCK] Writing {value} to pin {pin}")
            pass
            
        def gpio_read(self, chip, pin):
            # Simulate realistic ultrasonic sensor behavior
            return random.choice([0, 1])
            
        def gpio_free(self, chip, pin):
            print(f"[MOCK] Freeing pin {pin} on chip {chip}")
            self.claimed_pins.discard(pin)
            
        def gpiochip_close(self, chip):
            print(f"[MOCK] Closing GPIO chip {chip}")
    
    lgpio = MockLGPIO()

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
        'trig': 6,
        'echo': 16,
        'frame_id': 'ultrasonic_right',
        'field_of_view': 0.26,
        'angle': -120  # degrees from front
    }
}

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        self.chip = None
        self.claimed_pins = []
        
        # Initialize GPIO chip
        if GPIO_AVAILABLE:
            try:
                self.chip = lgpio.gpiochip_open(0)
                self.get_logger().info(f'✅ Opened GPIO chip 0: handle={self.chip}')
            except Exception as e:
                self.get_logger().error(f'❌ Failed to open GPIO chip: {e}')
                if os.geteuid() != 0:
                    self.get_logger().error("💡 Try running with sudo for GPIO access")
                raise
        else:
            self.chip = 0
            self.get_logger().warn("⚠️  Using mock GPIO - install lgpio for real sensors")
        
        # Set up GPIO pins with individual error handling
        self.sensor_status = {}
        for sensor_name, config in SENSOR_CONFIG.items():
            try:
                if GPIO_AVAILABLE:
                    # Claim pins
                    lgpio.gpio_claim_output(self.chip, config['trig'])
                    lgpio.gpio_claim_input(self.chip, config['echo'])
                    
                    # Set trigger low initially
                    lgpio.gpio_write(self.chip, config['trig'], 0)
                    
                    # Track claimed pins for cleanup
                    self.claimed_pins.extend([config['trig'], config['echo']])
                
                self.sensor_status[sensor_name] = True
                self.get_logger().info(f'✅ {sensor_name} sensor configured (trig: {config["trig"]}, echo: {config["echo"]})')
                
            except Exception as e:
                self.sensor_status[sensor_name] = False
                self.get_logger().error(f'❌ Failed to setup {sensor_name} sensor: {e}')
        
        # Check if any sensors were configured successfully
        working_sensors = sum(self.sensor_status.values())
        if GPIO_AVAILABLE and working_sensors == 0:
            self.get_logger().error("No sensors could be configured!")
            if os.geteuid() != 0:
                self.get_logger().error("💡 Try running with: sudo -E ros2 run ultrasonic ultrasonic")
        
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
        
        # Status report
        status = f"with lgpio ({working_sensors}/3 sensors working)" if GPIO_AVAILABLE else "with mock GPIO (testing mode)"
        self.get_logger().info(f'🔊 Ultrasonic sensor node initialized {status}')
        
        self.get_logger().info('📡 Publishing to topics:')
        for sensor_name in SENSOR_CONFIG.keys():
            self.get_logger().info(f'  - /ultrasonic_{sensor_name}')
        self.get_logger().info('  - /ultrasonic_array')
    
    def measurement_loop(self, sensor_name):
        """Continuously measure distance for a specific sensor"""
        config = SENSOR_CONFIG[sensor_name]
        
        while self.running:
            try:
                # Check if this sensor was set up successfully
                if GPIO_AVAILABLE and self.sensor_status.get(sensor_name, False):
                    distance = self.measure_distance(config['trig'], config['echo'])
                else:
                    # Use mock measurement if GPIO failed or unavailable
                    distance = self.mock_measure_distance(sensor_name)
                
                # Update stored value
                with self.lock:
                    self.distances[sensor_name] = distance
                
                # Small delay between measurements
                time.sleep(0.05)
                
            except Exception as e:
                self.get_logger().error(f'Error in {sensor_name} sensor: {e}')
                # Fall back to mock measurement on error
                with self.lock:
                    self.distances[sensor_name] = self.mock_measure_distance(sensor_name)
                time.sleep(0.1)
    
    def measure_distance(self, trig_pin, echo_pin):
        """Measure distance using HC-SR04 ultrasonic sensor with lgpio"""
        try:
            # Send trigger pulse (10 microseconds)
            lgpio.gpio_write(self.chip, trig_pin, 1)
            time.sleep(0.00001)  # 10 microseconds
            lgpio.gpio_write(self.chip, trig_pin, 0)
            
            # Wait for echo to start
            pulse_start = time.time()
            timeout_start = pulse_start
            
            while lgpio.gpio_read(self.chip, echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > 0.1:  # 100ms timeout
                    return self.max_range
            
            # Wait for echo to end
            pulse_end = time.time()
            timeout_start = pulse_end
            
            while lgpio.gpio_read(self.chip, echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end - timeout_start > 0.1:  # 100ms timeout
                    return self.max_range
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound (343 m/s) / 2
            distance = distance / 100  # Convert cm to meters
            
            # Clamp to valid range
            distance = max(self.min_range, min(distance, self.max_range))
            
            return distance
            
        except Exception as e:
            self.get_logger().error(f'GPIO measurement error on pins {trig_pin}/{echo_pin}: {e}')
            return self.max_range
    
    def mock_measure_distance(self, sensor_name):
        """Mock distance measurement for testing"""
        # Simulate realistic distance readings with some variation
        base_distances = {'left': 1.5, 'center': 2.0, 'right': 1.8}
        base = base_distances.get(sensor_name, 2.0)
        
        # Add some random variation to make it look realistic
        variation = random.uniform(-0.3, 0.3)
        distance = base + variation
        
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
        
        # Log occasionally (every 5 seconds)
        if int(current_time.nanoseconds / 1e9) % 5 == 0:
            working_sensors = sum(self.sensor_status.values()) if GPIO_AVAILABLE else 0
            mode_str = f" ({working_sensors}/3 real)" if GPIO_AVAILABLE else " (MOCK)"
            self.get_logger().info(
                f'📊 Distances{mode_str} - L: {current_distances["left"]:.2f}m, '
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
        if GPIO_AVAILABLE and self.chip is not None:
            try:
                # Free all claimed pins
                for pin in self.claimed_pins:
                    try:
                        lgpio.gpio_free(self.chip, pin)
                    except Exception as e:
                        self.get_logger().warn(f'Could not free pin {pin}: {e}')
                
                # Close the chip
                lgpio.gpiochip_close(self.chip)
                self.get_logger().info('✅ GPIO cleaned up successfully')
                
            except Exception as e:
                self.get_logger().error(f'Error during GPIO cleanup: {e}')
        
        self.get_logger().info('🧹 Ultrasonic sensor node cleaned up')

def main(args=None):
    """Main function"""
    print("🔊 Ultrasonic Sensor ROS2 Node Starting...")
    print("📡 3 HC-SR04 sensors configured at 120° intervals")
    print("🍓 Raspberry Pi 5 + Ubuntu 24.04 + lgpio")
    
    # Check permissions
    if not GPIO_AVAILABLE:
        print("⚠️  lgpio not available - install it first!")
        print("💡 Run: sudo apt install python3-lgpio")
    elif os.geteuid() != 0:
        print("⚠️  Not running as root - GPIO may fail")
        print("💡 If you get GPIO errors, try: sudo -E ros2 run ultrasonic ultrasonic")
    else:
        print("✅ Running with root privileges")
    
    print("=" * 50)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and run node
        ultrasonic_node = UltrasonicSensorNode()
        
        print("✅ Ultrasonic sensors initialized")
        print("📊 Publishing sensor data to ROS2 topics...")
        print("🔍 Use 'ros2 topic echo /ultrasonic_center' to see data")
        print("⏹️  Press Ctrl+C to stop")
        
        # Spin the node
        rclpy.spin(ultrasonic_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
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