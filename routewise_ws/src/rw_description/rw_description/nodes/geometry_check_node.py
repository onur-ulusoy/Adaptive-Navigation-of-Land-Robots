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

        # Construct the path to your xacro file
        xacro_path = os.path.join(pkg_path, 'share', 'rw_description', 'urdf', 'routewise.urdf.xacro')
        urdf_path = os.path.join(pkg_path, 'share', 'rw_description', 'urdf', 'routewise.urdf')

        # Run xacro command
        xacro_result = subprocess.run(['xacro', xacro_path], stdout=subprocess.PIPE, text=True)
        if xacro_result.returncode != 0:
            self.get_logger().error(f'Xacro command failed with error: {xacro_result.stderr}')
            return

        # Save the xacro output to a urdf file
        with open(urdf_path, 'w') as urdf_file:
            urdf_file.write(xacro_result.stdout)

        # Run check_urdf command
        check_urdf_result = subprocess.run(['check_urdf', urdf_path], stdout=subprocess.PIPE, text=True)
        if check_urdf_result.returncode == 0:
            self.get_logger().info(f'check_urdf output: {check_urdf_result.stdout}')
        else:
            self.get_logger().error(f'check_urdf command failed with error: {check_urdf_result.stderr}')

        # Shutdown the node after executing the commands
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GeometryCheckNode()

if __name__ == '__main__':
    main()
