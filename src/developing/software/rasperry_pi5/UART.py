#!/usr/bin/env python3
"""
UART communication bridge between ROS2 and motor controller
- Sends velocity commands from /cmd_vel to motors
- Handles emergency stop feedback to prevent velocity command conflicts
"""
#0801
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import threading
import time
import math
import numpy as np

class MotorUARTBridge(Node):
    def __init__(self):
        super().__init__('motor_uart_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_frequency', 20.0)  # Hz
        
        # Robot kinematics parameters
        self.radius_base = 110.0  # mm
        self.forward_kinematics = np.array([[0, 1, -self.radius_base], 
                                            [-math.sin(math.pi / 3), -math.cos(math.pi / 3), -self.radius_base], 
                                            [math.cos(math.pi / 6), -math.sin(math.pi / 6), -self.radius_base]])
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        # Emergency stop state
        self.emergency_stop_active = False
        self.last_cmd_vel = Twist()  # Store last command
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for emergency stop status (for other nodes to know)
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )
        
        # Open serial port
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Serial port opened on {serial_port} at {baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None
            return
        
        # Start receiver thread
        self.running = True
        self.receiver_thread = threading.Thread(target=self.receive_loop)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        self.get_logger().info("Motor UART bridge initialized with emergency stop handling")
        self.get_logger().info(f"Control frequency: {self.get_parameter('control_frequency').value} Hz")

    def cmd_vel_to_motor_velocities(self, vx_mm, vy_mm, angular_z):
        """
        Convert cmd_vel to individual motor velocities.
        
        Args:
            vx_mm: Forward velocity in mm/s
            vy_mm: Lateral velocity in mm/s (for holonomic robots)
            angular_z: Angular velocity in rad/s
            
        Returns:
            tuple: (vel1, vel2, vel3) in mm/s
        """
        command_matrix = np.array([[vx_mm], 
                                    [vy_mm], 
                                    [angular_z]])
        result = self.forward_kinematics @ command_matrix
        vel3 = result[0, 0]
        vel2 = result[1, 0]
        vel1 = result[2, 0]
        
        return vel1, vel2, vel3

    def cmd_vel_callback(self, msg):
        """
        Convert cmd_vel to motor velocities and send via UART
        """
        # Store the last command
        self.last_cmd_vel = msg
        
        # If emergency stop is active and there's forward velocity, set it to zero
        if self.emergency_stop_active:
            # Allow rotation and lateral movement, but no forward movement
            if msg.linear.x > 0:  # Only block forward movement
                self.get_logger().warn("Emergency stop active - blocking forward velocity")
                msg.linear.x = 0.0
            # Allow backward movement to escape
        
        # Convert cmd_vel to motor velocities using kinematics
        vel1, vel2, vel3 = self.cmd_vel_to_motor_velocities(
            msg.linear.x * 1000,  # Convert m/s to mm/s
            msg.linear.y * 1000,  # Convert m/s to mm/s
            msg.angular.z
        )
        
        # Send velocities via UART
        uart_message = f"{vel1:.2f},{vel2:.2f},{vel3:.2f}\n"
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(uart_message.encode('utf-8'))
                self.get_logger().debug(f"Sent: {uart_message.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send UART message: {e}")

    def publish_emergency_stop_status(self):
        """
        Publish emergency stop status
        """
        msg = Bool()
        msg.data = self.emergency_stop_active
        self.emergency_stop_pub.publish(msg)

    def receive_loop(self):
        """
        Continuously receive data from UART and process messages
        """
        buffer = ""
        
        while self.running and self.ser and self.ser.is_open:
            try:
                # Read available data
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self.process_uart_line(line.strip())
                        
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Unexpected error in receive loop: {e}")
                time.sleep(0.1)

    def process_uart_line(self, line):
        """
        Process a line received from UART
        """
        if not line:
            return
        
        # Handle emergency stop status
        if line.startswith("ESTOP:"):
            try:
                status = line[6:].strip()
                old_status = self.emergency_stop_active
                self.emergency_stop_active = (status == "ACTIVE")
                
                # Log status change
                if old_status != self.emergency_stop_active:
                    if self.emergency_stop_active:
                        self.get_logger().warn("EMERGENCY STOP ACTIVATED - Obstacle detected!")
                    else:
                        self.get_logger().info("Emergency stop cleared - Path is clear")
                    
                    # Publish status
                    self.publish_emergency_stop_status()
                    
            except Exception as e:
                self.get_logger().warning(f"Failed to parse emergency stop data: {line} - {e}")
        
        # Handle velocity data (for monitoring/debugging only)
        elif line.startswith("VEL:"):
            try:
                # Parse: "VEL:vel1,vel2,vel3,timestamp"
                data = line[4:].split(',')
                if len(data) >= 4:
                    motor_vels = [float(data[0]), float(data[1]), float(data[2])]
                    self.get_logger().debug(
                        f"Motor velocities: [{motor_vels[0]:.1f}, "
                        f"{motor_vels[1]:.1f}, {motor_vels[2]:.1f}] mm/s"
                    )
            except (ValueError, IndexError) as e:
                self.get_logger().warning(f"Failed to parse velocity data: {line} - {e}")
                
        # Handle acknowledgments
        elif line.startswith("ACK:"):
            self.get_logger().debug(f"Motor controller acknowledged: {line}")
            
        # Handle other messages
        else:
            self.get_logger().info(f"Received: {line}")

    def destroy_node(self):
        """
        Clean up resources
        """
        self.running = False
        if hasattr(self, 'receiver_thread'):
            self.receiver_thread.join(timeout=1.0)
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorUARTBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# #!/usr/bin/env python3
# """
# Bidirectional UART communication between ROS2 and motor controller
# - Sends velocity commands from /cmd_vel to motors
# - Receives motor velocities and calculates odometry by integration
# - Publishes full odometry with position to /motor_odometry
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Header
# # from communication.msg import MotorOdometry
# from communication_interfaces.msg import MotorOdometry
# import serial
# import threading
# import time
# import math
# import numpy as np

# class MotorUARTBridge(Node):
#     def __init__(self):
#         super().__init__('motor_uart_bridge')
        
#         # Parameters
#         self.declare_parameter('serial_port', '/dev/ttyAMA0')
#         self.declare_parameter('baud_rate', 115200)
#         self.declare_parameter('odometry_topic', '/motor_odometry')
#         self.declare_parameter('cmd_vel_topic', '/cmd_vel')
#         self.declare_parameter('control_frequency', 20.0)  # Hz
#         self.radius_base = 110.0 #mm
#         self.forward_kinematics = np.array([[0, 1, -self.radius_base], 
#                                             [-math.sin(math.pi / 3), -math.cos(math.pi / 3), -self.radius_base], 
#                                             [math.cos(math.pi / 6), -math.sin(math.pi / 6), -self.radius_base]])
#         self.inv_kinematics = np.linalg.inv(self.forward_kinematics)
#         serial_port = self.get_parameter('serial_port').value
#         baud_rate = self.get_parameter('baud_rate').value
#         self.control_period = 1.0 / self.get_parameter('control_frequency').value
        
#         # Robot state (position calculated by integration)
#         self.x = 0.0          # mm
#         self.y = 0.0          # mm
#         self.theta = 0.0      # radians
#         self.vx = 0.0         # mm/s
#         self.vy = 0.0         # mm/s
#         self.vtheta = 0.0     # rad/s
#         self.motor_velocities = [0.0, 0.0, 0.0]  # mm/s
#         self.last_velocity_time = None
        
#         # Subscriber for velocity commands
#         self.cmd_vel_sub = self.create_subscription(
#             Twist,
#             self.get_parameter('cmd_vel_topic').value,
#             self.cmd_vel_callback,
#             10
#         )
        
#         # Publisher for odometry data
#         # Uncomment when using custom message:
#         self.odometry_pub = self.create_publisher(
#             MotorOdometry,
#             self.get_parameter('odometry_topic').value,
#             10
#         )
        
#         # # For testing without custom message, publish as string
#         # from std_msgs.msg import String
#         # self.odometry_pub = self.create_publisher(
#         #     String,
#         #     self.get_parameter('odometry_topic').value,
#         #     10
#         # )
        
#         # Open serial port
#         try:
#             self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
#             self.get_logger().info(f"Serial port opened on {serial_port} at {baud_rate} baud")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
#             self.ser = None
#             return
        
#         # Start receiver thread
#         self.running = True
#         self.receiver_thread = threading.Thread(target=self.receive_loop)
#         self.receiver_thread.daemon = True
#         self.receiver_thread.start()
        
#         self.get_logger().info("Motor UART bridge initialized")
#         self.get_logger().info(f"Control frequency: {self.get_parameter('control_frequency').value} Hz")

#     def cmd_vel_to_motor_velocities(self, vx_mm, vy_mm, angular_z):
#         """
#         Convert cmd_vel to individual motor velocities.
        
#         Args:
#             vx_mm: Forward velocity in mm/s
#             vy_mm: Lateral velocity in mm/s (for holonomic robots)
#             angular_z: Angular velocity in rad/s
            
#         Returns:
#             tuple: (vel1, vel2, vel3) in mm/s
            
#         TODO: Implement this based on your robot's kinematics
#         """

        
#         # ============================================
#         # KINEMATICS
#         # ============================================

#         command_matrix = np.array([[vx_mm], 
#                                     [vy_mm], 
#                                     [angular_z]])
#         result = self.forward_kinematics @ command_matrix
#         vel3 = result[0, 0]
#         vel2 = result[1, 0]
#         vel1 = result[2, 0]
        
#         return vel1, vel2, vel3

#     def motor_velocities_to_robot_velocities(self, vel1, vel2, vel3):
#         """
#         Convert individual motor velocities to robot velocities (inverse kinematics).
        
#         Args:
#             vel1, vel2, vel3: Motor velocities in mm/s
            
#         Returns:
#             tuple: (vx, vy, vtheta) - robot velocities in mm/s and rad/s
            
#         TODO: Implement this based on your robot's kinematics
#         """
#         # ============================================
#         # IMPLEMENT YOUR INVERSE KINEMATICS HERE
#         # ============================================
#         actual = np.array([[vel3], [vel2], [vel1]])
#         _result = self.inv_kinematics @ actual
#         vx = _result[0, 0]
#         vy = _result[1, 0]
#         vtheta = _result[2, 0]

        

        
#         return vx, vy, vtheta

#     def cmd_vel_callback(self, msg):
#         """
#         Convert cmd_vel to motor velocities and send via UART
#         """
#         # Convert cmd_vel to motor velocities using kinematics
#         vel1, vel2, vel3 = self.cmd_vel_to_motor_velocities(
#             msg.linear.x, 
#             msg.linear.y, 
#             msg.angular.z
#         )
        
#         # Send velocities via UART
#         uart_message = f"{vel1:.2f},{vel2:.2f},{vel3:.2f}\n"
        
#         if self.ser and self.ser.is_open:
#             try:
#                 self.ser.write(uart_message.encode('utf-8'))
#                 self.get_logger().debug(f"Sent: {uart_message.strip()}")
#             except serial.SerialException as e:
#                 self.get_logger().error(f"Failed to send UART message: {e}")

#     def integrate_odometry(self, dt):
#         """
#         Integrate velocities to update robot position
        
#         Args:
#             dt: Time step in seconds
#         """
#         # Update position using current velocities
#         # For differential drive or any robot
#         self.x += self.vx * math.cos(self.theta) * dt - self.vy * math.sin(self.theta) * dt
#         self.y += self.vx * math.sin(self.theta) * dt + self.vy * math.cos(self.theta) * dt
#         self.theta += self.vtheta * dt
        
#         # Normalize theta to [-pi, pi]
#         while self.theta > math.pi:
#             self.theta -= 2 * math.pi
#         while self.theta < -math.pi:
#             self.theta += 2 * math.pi

#     def publish_odometry(self):
#         """
#         Publish odometry message
#         """
#         # When using custom message, uncomment this:
#         msg = MotorOdometry()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "odom"
#         msg.motor_velocities = self.motor_velocities
#         msg.x = self.x
#         msg.y = self.y
#         msg.theta = self.theta
#         msg.vx = self.vx
#         msg.vy = self.vy
#         msg.vtheta = self.vtheta
#         self.odometry_pub.publish(msg)
        
#         # For testing, publish as string
#         # from std_msgs.msg import String
#         # msg = String()
#         # msg.data = (f"Pose: x={self.x:.1f}mm, y={self.y:.1f}mm, theta={math.degrees(self.theta):.1f}deg | "
#         #            f"Vel: vx={self.vx:.1f}mm/s, vy={self.vy:.1f}mm/s, vtheta={math.degrees(self.vtheta):.1f}deg/s | "
#         #            f"Motors: {self.motor_velocities[0]:.1f}, {self.motor_velocities[1]:.1f}, {self.motor_velocities[2]:.1f} mm/s")
#         # self.odometry_pub.publish(msg)

#     def receive_loop(self):
#         """
#         Continuously receive data from UART and process odometry
#         """
#         buffer = ""
        
#         while self.running and self.ser and self.ser.is_open:
#             try:
#                 # Read available data
#                 if self.ser.in_waiting > 0:
#                     data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
#                     buffer += data
                    
#                     # Process complete lines
#                     while '\n' in buffer:
#                         line, buffer = buffer.split('\n', 1)
#                         self.process_uart_line(line.strip())
                        
#             except serial.SerialException as e:
#                 self.get_logger().error(f"Serial read error: {e}")
#                 time.sleep(0.1)
#             except Exception as e:
#                 self.get_logger().error(f"Unexpected error in receive loop: {e}")
#                 time.sleep(0.1)

#     def process_uart_line(self, line):
#         """
#         Process a line received from UART
#         """
#         if not line:
#             return
            
#         # Handle velocity data
#         if line.startswith("VEL:"):
#             try:
#                 # Parse: "VEL:vel1,vel2,vel3,timestamp"
#                 data = line[4:].split(',')
#                 if len(data) >= 4:
#                     # Extract motor velocities
#                     self.motor_velocities = [float(data[0]), float(data[1]), float(data[2])]
#                     timestamp_ms = int(data[3])
                    
#                     # Calculate robot velocities from motor velocities
#                     self.vx, self.vy, self.vtheta = self.motor_velocities_to_robot_velocities(
#                         self.motor_velocities[0],
#                         self.motor_velocities[1], 
#                         self.motor_velocities[2]
#                     )
                    
#                     # Integrate position if we have a previous time
#                     current_time = timestamp_ms / 1000.0  # Convert to seconds
#                     if self.last_velocity_time is not None:
#                         dt = current_time - self.last_velocity_time
#                         if dt > 0 and dt < 1.0:  # Sanity check
#                             self.integrate_odometry(dt)
                    
#                     self.last_velocity_time = current_time
                    
#                     # Publish odometry
#                     self.publish_odometry()
                    
#                     self.get_logger().debug(
#                         f"Motor vels: [{self.motor_velocities[0]:.1f}, "
#                         f"{self.motor_velocities[1]:.1f}, {self.motor_velocities[2]:.1f}] mm/s"
#                     )
                    
#             except (ValueError, IndexError) as e:
#                 self.get_logger().warning(f"Failed to parse velocity data: {line} - {e}")
                
#         # Handle acknowledgments
#         elif line.startswith("ACK:"):
#             self.get_logger().debug(f"Motor controller acknowledged: {line}")
            
#         # Handle other messages
#         else:
#             self.get_logger().info(f"Received: {line}")

#     def destroy_node(self):
#         """
#         Clean up resources
#         """
#         self.running = False
#         if hasattr(self, 'receiver_thread'):
#             self.receiver_thread.join(timeout=1.0)
#         if hasattr(self, 'ser') and self.ser and self.ser.is_open:
#             self.ser.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorUARTBridge()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
