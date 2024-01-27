#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "turtlesim_controller/turtlesim_controller.hpp"

using namespace std::chrono_literals;

TurtlesimController::TurtlesimController() : Node("turtlesim_controller")
{
  num_of_sides_ = this->declare_parameter<int>("num_of_sides", 3);
  length_of_side_ = this->declare_parameter<double>("length_of_side", 1.0);
  turn_direction_th_ = this->declare_parameter<double>("turn_direction_th", 0.01);
  turtle_.velocity = this->declare_parameter<double>("velocity", 0.5);
  turtle_.yawrate = this->declare_parameter<double>("yawrate", 0.3);

  RCLCPP_INFO(this->get_logger(), "turtle controller started");
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/turtle1/cmd_vel", rclcpp::QoS(1).reliable());
  pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
    "/turtle1/pose", rclcpp::QoS(1).reliable(),
    std::bind(&TurtlesimController::pose_callback, this, std::placeholders::_1));
}

void TurtlesimController::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  turtle_.pose = *msg;
  set_cmd_vel();
}

void TurtlesimController::set_cmd_vel()
{
  geometry_msgs::msg::Twist cmd_vel;
  if (!can_move())
  {
    cmd_vel_pub_->publish(cmd_vel);
    return;
  }

  if (!prev_turn_pose_.has_value())
    prev_turn_pose_ = turtle_.pose.value();
  const double distance = calc_distance(prev_turn_pose_.value(), turtle_.pose.value());

  // go_straight or turn
  if (distance < length_of_side_)
  {
    cmd_vel = get_cmd_vel_to_go_straight();
  }
  else
  {
    if (can_turn())
    {
      cmd_vel = get_cmd_vel_to_turn_in_place();
    }
    else
    {
      turn_count_++;
      prev_turn_pose_.reset();
    }
  }

  print_status();
  cmd_vel_pub_->publish(cmd_vel);
}

bool TurtlesimController::can_move() { return turtle_.pose.has_value() && turn_count_ < num_of_sides_; }

double TurtlesimController::calc_distance(const turtlesim::msg::Pose pose1, const turtlesim::msg::Pose pose2)
{
  return hypot(pose1.x - pose2.x, pose1.y - pose2.y);
}

geometry_msgs::msg::Twist TurtlesimController::get_cmd_vel_to_go_straight()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = turtle_.velocity;

  return cmd_vel;
}

geometry_msgs::msg::Twist TurtlesimController::get_cmd_vel_to_turn_in_place()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.angular.z = turtle_.yawrate;

  return cmd_vel;
}

bool TurtlesimController::can_turn()
{
  return turn_direction_th_ <
             abs(calc_target_direction(turtle_.pose.value(), turn_count_) - turtle_.pose.value().theta) &&
         turn_count_ < num_of_sides_ - 1;
}

double TurtlesimController::calc_target_direction(const turtlesim::msg::Pose pose, const int turn_count)
{
  const double target_direction_base = 2.0 * M_PI / num_of_sides_;
  double target_direction = target_direction_base * (turn_count_ + 1);
  while (M_PI < target_direction)
    target_direction -= 2.0 * M_PI;

  return target_direction;
}

void TurtlesimController::print_status()
{
  if (!turtle_.pose.has_value())
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "turtle_.pose.has_value: " << turtle_.pose.has_value());
    return;
  }
  if (!prev_turn_pose_.has_value())
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "prev_turn_pose_.has_value: " << prev_turn_pose_.has_value());
    return;
  }

  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "---");
  RCLCPP_INFO(this->get_logger(), "current_pose:");
  RCLCPP_INFO_STREAM(this->get_logger(), "\tx: " << turtle_.pose.value().x);
  RCLCPP_INFO_STREAM(this->get_logger(), "\ty: " << turtle_.pose.value().y);
  RCLCPP_INFO_STREAM(this->get_logger(), "\ttheta: " << turtle_.pose.value().theta);
  RCLCPP_INFO_STREAM(this->get_logger(), "\tlinear_velocity: " << turtle_.pose.value().linear_velocity);
  RCLCPP_INFO_STREAM(this->get_logger(), "\tangular_velocity: " << turtle_.pose.value().angular_velocity);
  RCLCPP_INFO(this->get_logger(), "prev_turn_pose:");
  RCLCPP_INFO_STREAM(this->get_logger(), "\tx: " << prev_turn_pose_.value().x);
  RCLCPP_INFO_STREAM(this->get_logger(), "\ty: " << prev_turn_pose_.value().y);
  RCLCPP_INFO_STREAM(this->get_logger(), "\ttheta: " << prev_turn_pose_.value().theta);
  RCLCPP_INFO_STREAM(this->get_logger(), "\tlinear_velocity: " << prev_turn_pose_.value().linear_velocity);
  RCLCPP_INFO_STREAM(this->get_logger(), "\tangular_velocity: " << prev_turn_pose_.value().angular_velocity);
  RCLCPP_INFO_STREAM(this->get_logger(),
      "distance from last turning position: " << calc_distance(turtle_.pose.value(), prev_turn_pose_.value()));
  RCLCPP_INFO_STREAM(this->get_logger(),
      "diff from target direction: " << abs(
          calc_target_direction(turtle_.pose.value(), turn_count_) - turtle_.pose.value().theta));
}