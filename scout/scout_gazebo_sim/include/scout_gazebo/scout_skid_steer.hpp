#ifndef SCOUT_SKID_STEER_HPP
#define SCOUT_SKID_STEER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp" // Add this include
#include <string>

namespace wescore {
class ScoutSkidSteer : public rclcpp::Node {
 public:
  // Constructor (ROS 2 version, no NodeHandle)
  explicit ScoutSkidSteer(std::string robot_name = "");

  // Method to setup the subscription
  void SetupSubscription();

 private:
  std::string robot_name_;
  std::string motor_fr_topic_;
  std::string motor_fl_topic_;
  std::string motor_rl_topic_;
  std::string motor_rr_topic_;
  std::string cmd_topic_;

  const double SCOUT_WHEELBASE = 0.498;
  const double SCOUT_WHEEL_RADIUS = 0.16459;

  // ROS 2 publisher and subscriber
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_fr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_fl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rr_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  // Callback function for the Twist command
  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
}  // namespace wescore

#endif /* SCOUT_SKID_STEER_HPP */