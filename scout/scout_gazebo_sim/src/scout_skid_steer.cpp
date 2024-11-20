#include "scout_gazebo/scout_skid_steer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/rclcpp.hpp"  // Ensure you include rclcpp

namespace wescore {

ScoutSkidSteer::ScoutSkidSteer(std::string robot_name)
    : rclcpp::Node("scout_skid_steer"), robot_name_(robot_name) {

  motor_fr_topic_ = robot_name_ + "scout_motor_fr_controller/command";
  motor_fl_topic_ = robot_name_ + "scout_motor_fl_controller/command";
  motor_rl_topic_ = robot_name_ + "scout_motor_rl_controller/command";
  motor_rr_topic_ = robot_name_ + "scout_motor_rr_controller/command";
  std::string robot_name_ = "scout";
  cmd_topic_ = robot_name_ + "/cmd_vel";

  // Create the publishers for the motor command topics
  motor_fr_pub_ = this->create_publisher<std_msgs::msg::Float64>(motor_fr_topic_, 10);
  motor_fl_pub_ = this->create_publisher<std_msgs::msg::Float64>(motor_fl_topic_, 10);
  motor_rl_pub_ = this->create_publisher<std_msgs::msg::Float64>(motor_rl_topic_, 10);
  motor_rr_pub_ = this->create_publisher<std_msgs::msg::Float64>(motor_rr_topic_, 10);

  // Setup the subscriber for the Twist command topic
  SetupSubscription();
}

void ScoutSkidSteer::SetupSubscription() {
  // Subscribe to the cmd_vel topic with Twist messages
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10, std::bind(&ScoutSkidSteer::TwistCmdCallback, this, std::placeholders::_1));
}

void ScoutSkidSteer::TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std_msgs::msg::Float64 motor_cmd[4];

  double driving_vel = msg->linear.x;
  double steering_vel = msg->angular.z;

  // Compute the velocities for the left and right sides of the skid steer
  double left_side_velocity =
      (driving_vel - steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;
  double right_side_velocity =
      (driving_vel + steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;

  motor_cmd[0].data = right_side_velocity;  // Right front motor
  motor_cmd[1].data = -left_side_velocity;  // Left front motor
  motor_cmd[2].data = -left_side_velocity;  // Left rear motor
  motor_cmd[3].data = right_side_velocity;  // Right rear motor

  // Publish the computed motor velocities to their respective topics
  motor_fr_pub_->publish(motor_cmd[0]);
  motor_fl_pub_->publish(motor_cmd[1]);
  motor_rl_pub_->publish(motor_cmd[2]);
  motor_rr_pub_->publish(motor_cmd[3]);
}

}  // namespace wescore