#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include <string>

#include "scout_gazebo/scout_skid_steer.hpp"

using namespace wescore;

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the ROS 2 Node
  std::string robot_namespace = "scout_default";
  
  // Create node with namespace parameter
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("scout_odom");

  // Fetch parameter from the parameter server
  node->declare_parameter<std::string>("robot_namespace", robot_namespace);
  node->get_parameter("robot_namespace", robot_namespace);

  RCLCPP_INFO(node->get_logger(), "Namespace: %s", robot_namespace.c_str());

  // Create the ScoutSkidSteer controller object
  wescore::ScoutSkidSteer skid_steer_controller(robot_namespace);
  skid_steer_controller.SetupSubscription();

  // Spin the node to process callbacks
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}