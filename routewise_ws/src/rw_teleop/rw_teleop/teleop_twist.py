import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_x = 0.5
        self.linear_y = 0.5
        self.angular_z = 1.0

        self.get_logger().info('Ready for keyboard input...')
        self.run()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)  # read 1 character
        if key == '\x1b':  # if it's an escape character
            key += sys.stdin.read(2)  # read the next two characters
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while rclpy.ok():
            key = self.getKey()
            twist = Twist()

            if key == 'q':
                self.linear_x += 0.1
            elif key == 'w':
                self.linear_x -= 0.1
            elif key == 'a':
                self.linear_y += 0.1
            elif key == 's':
                self.linear_y -= 0.1
            elif key == 'z':
                self.angular_z += 0.1
            elif key == 'x':
                self.angular_z -= 0.1
            elif key == '\x1b[A':  # UP arrow
                twist.linear.x = self.linear_x
            elif key == '\x1b[B':  # DOWN arrow
                twist.linear.x = -self.linear_x
            elif key == '\x1b[C':  # RIGHT arrow
                twist.linear.y = -self.linear_y
            elif key == '\x1b[D':  # LEFT arrow
                twist.linear.y = self.linear_y
            elif key == 'l':
                twist.angular.z = -self.angular_z
            elif key == 'j':
                twist.angular.z = self.angular_z
            elif key == 'k':
                # Stop everything
                twist.linear.x = 0
                twist.linear.y = 0
                twist.angular.z = 0
            else:
                self.get_logger().info('Invalid key')
                continue

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_keyboard = TeleopTwistKeyboard()
    rclpy.spin(teleop_twist_keyboard)
    teleop_twist_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
