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

        # Open UART over GPIO using /dev/serial0
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)  # GPIO14 (TX), GPIO15 (RX)
            self.get_logger().info("Serial port opened on /dev/serial0 (GPIO14/15)")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        uart_message = f"{linear:.2f},{angular:.2f}\n"

        if self.ser and self.ser.is_open:
            self.ser.write(uart_message.encode('utf-8'))
            self.get_logger().info(f"Sent over UART: {uart_message.strip()}")
        else:
            self.get_logger().warn("Serial port not available")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelUART()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
