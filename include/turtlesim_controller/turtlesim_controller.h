#ifndef TURTLESIM_CONTROLLER_H
#define TURTLESIM_CONTROLLER_H

#include <optional>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <turtlesim/Pose.h>

struct Turtle
{
  std::optional<turtlesim::Pose> pose;
  double velocity;
  double yawrate;
};

class TurtlesimController
{
public:
  TurtlesimController(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
  void pose_callback(const turtlesim::Pose::ConstPtr &msg);
  void set_cmd_vel();
  void print_status();
  bool can_move();
  bool can_turn();
  double calc_target_direction(const turtlesim::Pose pose, const int turn_count);
  double calc_distance(const turtlesim::Pose pose1, const turtlesim::Pose pose2);
  geometry_msgs::Twist get_cmd_vel_to_go_straight();
  geometry_msgs::Twist get_cmd_vel_to_turn_in_place();

  int num_of_sides_ = 0;
  int turn_count_ = 0;
  double length_of_side_ = 0.0;
  double turn_direction_th_ = 0.0;

  Turtle turtle_;
  std::optional<turtlesim::Pose> prev_turn_pose_;

  ros::Subscriber pose_sub_;
  ros::Publisher cmd_vel_pub_;
};

#endif
