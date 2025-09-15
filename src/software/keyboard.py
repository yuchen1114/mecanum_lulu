'''
listen to keyboard and send cmd_vel to /cmd_vel
'''
# Import ROS 2 Python client library
import rclpy
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for velocity

# For reading keyboard input without Enter
import sys, termios, tty

class KeyboardTeleop(Node):
    def __init__(self):
        # Initialize node with name 'keyboard_teleop'
        super().__init__('keyboard_teleop')

        # Create publisher to /cmd_vel topic with Twist messages, queue size 10
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Show startup message
        self.get_logger().info("Use W/A/S/D to move, k to stop, Q to quit")

        # Start reading keys
        self.run()

    def get_key(self):
        # Save current terminal settings
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)  # Set terminal to raw mode (no Enter needed)
            ch = sys.stdin.read(1)  # Read one character
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)  # Restore settings
        return ch

    def run(self):
        speed = 1.0  # linear speed (mm/s)
        turn = 1.0   # angular speed (rad/s)

        while rclpy.ok():
            key = self.get_key()  # wait for one key press
            msg = Twist()  # create a new Twist message

            if key.lower() == 'w':
                msg.linear.x = speed
                msg.angular.z = 0.0  # move forward
            elif key.lower() == 's':
                msg.linear.x = -speed  
                msg.angular.z = 0.0  # move backward
            elif key.lower() == 'a':
                msg.linear.x = 0.0
                msg.angular.z = turn  # turn left
            elif key.lower() == 'd':
                msg.linear.x = 0.0
                msg.angular.z = -turn  # turn right
            elif key.lower() == 'k':
                msg.linear.x = 0.0
                msg.angular.z = 0.0  # turn right
            elif key.lower() == 'q':
                break  # exit the loop (and node)
            else:
                continue  # ignore other keys

            self.pub.publish(msg)  # publish the velocity command

            # Log the current velocities to the terminal
            self.get_logger().info(
                f"Sent: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python system
    node = KeyboardTeleop()  # Create and start the node
    rclpy.shutdown()  # Shutdown cleanly after exiting loop

if __name__ == '__main__':
    main()  # Entry point of the script

