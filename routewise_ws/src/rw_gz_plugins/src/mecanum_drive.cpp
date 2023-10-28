#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo {

class MecanumDrivePlugin : public ModelPlugin {

public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    model = _model;
    node = transport::NodePtr(new transport::Node());
    node->Init(model->GetWorld()->Name());
    
    // Initialize ROS 2 node
    rclcpp::init(0, nullptr);
    ros_node = rclcpp::Node::make_shared("mecanum_drive_plugin");

    // Get the wheel joints
    front_left_wheel = model->GetJoint("front_left_wheel_joint");
    front_right_wheel = model->GetJoint("front_right_wheel_joint");
    rear_left_wheel = model->GetJoint("rear_left_wheel_joint");
    rear_right_wheel = model->GetJoint("rear_right_wheel_joint");

    // Ensure the joints are available
    if (!front_left_wheel || !front_right_wheel || !rear_left_wheel || !rear_right_wheel) {
      RCLCPP_FATAL(ros_node->get_logger(), "Missing wheel joint(s)");
      return;
    }

    // Setup ROS 2 subscription
    cmd_vel_sub = ros_node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MecanumDrivePlugin::OnCmdVel, this, std::placeholders::_1)
    );

    // Connect to the world update event
    update_connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MecanumDrivePlugin::OnUpdate, this));
  }

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vx = msg->linear.x;
    vy = msg->linear.y;
    wz = msg->angular.z;
  }

  void OnUpdate() {
    // Implement Mecanum kinematics to compute wheel velocities
    double front_left_vel = vx - vy - wz;
    double front_right_vel = vx + vy + wz;
    double rear_left_vel = vx - vy + wz;
    double rear_right_vel = vx + vy - wz;

    // Set wheel velocities
    front_left_wheel->SetVelocity(0, front_left_vel);
    front_right_wheel->SetVelocity(0, front_right_vel);
    rear_left_wheel->SetVelocity(0, rear_left_vel);
    rear_right_wheel->SetVelocity(0, rear_right_vel);
  }

private:
  transport::NodePtr node;
  physics::ModelPtr model;
  physics::JointPtr front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Node::SharedPtr ros_node;
  event::ConnectionPtr update_connection;
  double vx = 0, vy = 0, wz = 0;

};

GZ_REGISTER_MODEL_PLUGIN(MecanumDrivePlugin)

}  // namespace gazebo
