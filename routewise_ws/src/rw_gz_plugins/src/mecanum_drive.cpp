#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo/gazebo.hh>

#include <iostream>
using namespace std;

namespace gazebo
{
  class MecanumDrive : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      cout << "MecanumDrive plugin is loaded." << endl;

      // Store the model pointer
      this->model = _parent;
      
      // Create ROS node
      this->node = std::make_shared<rclcpp::Node>("mecanum_drive");
      
      // Get the wheel joints
      front_left_wheel = model->GetJoint("front_left_wheel_joint");
      front_right_wheel = model->GetJoint("front_right_wheel_joint");
      rear_left_wheel = model->GetJoint("rear_left_wheel_joint");
      rear_right_wheel = model->GetJoint("rear_right_wheel_joint");

      // Ensure the joints are available
      if (!front_left_wheel || !front_right_wheel || !rear_left_wheel || !rear_right_wheel) {
        cout << "Missing wheel joint(s)" << endl;
        return;
      }

      // Get robot dimensions from SDF
      if (_sdf->HasElement("robotWidth")) {
        robot_width = _sdf->Get<double>("robotWidth");
      } else {
          cout << "No robotWidth specified. Using default value of 0.5." << endl;
          robot_width = 0.5;
      }

      if (_sdf->HasElement("robotLength")) {
        robot_length = _sdf->Get<double>("robotLength");
      } else {
          cout << "No robotLength specified. Using default value of 0.4." << endl;
          robot_length = 0.4;
      }

      // Setup ROS 2 subscription
      this->cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MecanumDrive::OnCmdVel, this, std::placeholders::_1)
      );
          
      // Initialize update connection
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MecanumDrive::OnUpdate, this));
    }

    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
      vx = msg->linear.x;
      vy = msg->linear.y;
      wz = msg->angular.z;
    }

    public: void OnUpdate()
    {
      double v_rot = wz * ((robot_width + robot_length) / 2);
      
      // Implement Mecanum kinematics to compute wheel velocities
      double front_left_vel = vx - vy - v_rot;
      double front_right_vel = vx + vy + v_rot;
      double rear_left_vel = vx + vy - v_rot;
      double rear_right_vel = vx - vy + v_rot;

      // Set wheel velocities
      front_left_wheel->SetVelocity(0, front_left_vel * 14.29);
      front_right_wheel->SetVelocity(0, front_right_vel * 14.29);
      rear_left_wheel->SetVelocity(0, rear_left_vel * 14.29);
      rear_right_wheel->SetVelocity(0, rear_right_vel * 14.29);

      rclcpp::spin_some(this->node);

    }

    private:

    rclcpp::Node::SharedPtr node;
    physics::ModelPtr model;
    physics::JointPtr front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    event::ConnectionPtr updateConnection;
    double vx = 0, vy = 0, wz = 0;
    double robot_width, robot_length;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MecanumDrive)
}
