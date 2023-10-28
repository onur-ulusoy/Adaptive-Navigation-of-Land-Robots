import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import signal

class TeleopTwistKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self._terminate = False

        self.linear_x = 0.5
        self.linear_y = 0.5
        self.angular_z = 1.0

        signal.signal(signal.SIGINT, self.signal_handler)

        self.get_logger().info('Ready for keyboard input...')
        self.run()

    def signal_handler(self, sig, frame):
        """Handles keyboard interrupt, Ctrl+C"""
        self.get_logger().info('Exiting...')
        self._terminate = True
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        sys.exit(0)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while not self._terminate:
            key = self.getKey()
            twist = Twist()
            should_publish = True

            if key == 'q':
                self.linear_x += 0.05
                should_publish = False
            elif key == 'w':
                self.linear_x -= 0.05
                should_publish = False
            elif key == 'a':
                self.linear_y += 0.05
                should_publish = False
            elif key == 's':
                self.linear_y -= 0.05
                should_publish = False
            elif key == 'z':
                self.angular_z += 0.05
                should_publish = False
            elif key == 'x':
                self.angular_z -= 0.05
                should_publish = False
            elif key == '\x1b[A':  # UP arrow
                twist.linear.x = self.linear_x
            elif key == '\x1b[B':  # DOWN arrow
                twist.linear.x = -self.linear_x
            elif key == '\x1b[C':  # RIGHT arrow
                twist.linear.y = self.linear_y
            elif key == '\x1b[D':  # LEFT arrow
                twist.linear.y = -self.linear_y
            elif key == 'l':
                twist.angular.z = self.angular_z
            elif key == 'j':
                twist.angular.z = -self.angular_z
            elif key == 'k':
                # Stop everything
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
            elif key == '\x03':  # Ctrl+C
                self.get_logger().info('Exiting...')
                break
            else:
                self.get_logger().info('Invalid key')
                should_publish = False

            if should_publish:
                self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_keyboard = TeleopTwistKeyboard()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
