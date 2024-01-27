#ifndef TURTLESIM_CONTROLLER_HPP
#define TURTLESIM_CONTROLLER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

struct Turtle
{
  std::optional<turtlesim::msg::Pose> pose;
  double velocity;
  double yawrate;
};

class TurtlesimController : public rclcpp::Node
{
public:
  // TurtlesimController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  TurtlesimController();
  void process();

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
  void set_cmd_vel();
  void print_status();
  bool can_move();
  bool can_turn();
  double calc_target_direction(const turtlesim::msg::Pose pose, const int turn_count);
  double calc_distance(const turtlesim::msg::Pose pose1, const turtlesim::msg::Pose pose2);
  geometry_msgs::msg::Twist get_cmd_vel_to_go_straight();
  geometry_msgs::msg::Twist get_cmd_vel_to_turn_in_place();

  int num_of_sides_ = 0;
  int turn_count_ = 0;
  double length_of_side_ = 0.0;
  double turn_direction_th_ = 0.0;

  Turtle turtle_;
  std::optional<turtlesim::msg::Pose> prev_turn_pose_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

#endif
