'''
sending msg to MCU through UART
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelUART(Node):
    def __init__(self):
        super().__init__('cmd_vel_uart')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        # Open UART over GPIO using /dev/ttyAMA0
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)  # GPIO14 (TX), GPIO15 (RX)
            self.get_logger().info("Serial port opened on /dev/ttyAMA0 (GPIO14/15)")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular = msg.angular.z
        uart_message = f"{linear_x:.2f},{linear_y:.2f},{angular:.2f}\n"

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(uart_message.encode('utf-8'))
                self.get_logger().info(f"Sent over UART: {uart_message.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send UART message: {e}")
        else:
            self.get_logger().warn("Serial port not available")

    def __del__(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelUART()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()