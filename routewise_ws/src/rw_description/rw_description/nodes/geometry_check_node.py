import rclpy
from rclpy.node import Node
import subprocess
import os

class GeometryCheckNode(Node):
    def __init__(self):
        super().__init__('geometry_check_node')
        self.check_geometry()

    def check_geometry(self):
        # Obtain the package path
        pkg_path_process = subprocess.Popen(['ros2', 'pkg', 'prefix', 'rw_description'],
                                            stdout=subprocess.PIPE,
                                            text=True)
        pkg_path_output, _ = pkg_path_process.communicate()
        pkg_path = pkg_path_output.strip()  # Remove newline at the end

        # Construct the path to your script
        script_path = os.path.join(pkg_path, 'share', 'rw_description', 'urdf', 'geometry_check.bash')

        # Run the script
        result = subprocess.run([script_path], stdout=subprocess.PIPE, text=True)
        if result.returncode == 0:
            self.get_logger().info(f'Script output: {result.stdout}')
        else:
            self.get_logger().error(f'Script failed with error: {result.stderr}')

        # Shutdown the node after executing the script
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GeometryCheckNode()

if __name__ == '__main__':
    main()
