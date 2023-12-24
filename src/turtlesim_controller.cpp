#include "turtlesim_controller/turtlesim_controller.h"

TurtlesimController::TurtlesimController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  pnh.param<int>("num_of_sides", num_of_sides_, 3);
  pnh.param<double>("length_of_side", length_of_side_, 1.0);
  pnh.param<double>("turn_direction_th", turn_direction_th_, 0.01);
  pnh.param<double>("velocity", turtle_.velocity, 0.5);
  pnh.param<double>("yawrate", turtle_.yawrate, 0.3);

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  pose_sub_ = nh.subscribe("/turtle1/pose", 1, &TurtlesimController::pose_callback, this);
}

void TurtlesimController::pose_callback(const turtlesim::Pose::ConstPtr &msg)
{
  turtle_.pose = *msg;
  set_cmd_vel();
}

void TurtlesimController::process() { ros::spin(); }

void TurtlesimController::set_cmd_vel()
{
  geometry_msgs::Twist cmd_vel;
  if (!can_move())
  {
    cmd_vel_pub_.publish(cmd_vel);
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
  cmd_vel_pub_.publish(cmd_vel);
}

bool TurtlesimController::can_move() { return turtle_.pose.has_value() && turn_count_ < num_of_sides_; }

double TurtlesimController::calc_distance(const turtlesim::Pose pose1, const turtlesim::Pose pose2)
{
  return hypot(pose1.x - pose2.x, pose1.y - pose2.y);
}

geometry_msgs::Twist TurtlesimController::get_cmd_vel_to_go_straight()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = turtle_.velocity;

  return cmd_vel;
}

geometry_msgs::Twist TurtlesimController::get_cmd_vel_to_turn_in_place()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = turtle_.yawrate;

  return cmd_vel;
}

bool TurtlesimController::can_turn()
{
  return turn_direction_th_ <
             abs(calc_target_direction(turtle_.pose.value(), turn_count_) - turtle_.pose.value().theta) &&
         turn_count_ < num_of_sides_ - 1;
}

double TurtlesimController::calc_target_direction(const turtlesim::Pose pose, const int turn_count)
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
    ROS_WARN_STREAM("turtle_.pose.has_value: " << turtle_.pose.has_value());
    return;
  }
  if (!prev_turn_pose_.has_value())
  {
    ROS_WARN_STREAM("prev_turn_pose_.has_value: " << prev_turn_pose_.has_value());
    return;
  }

  ROS_INFO(" ");
  ROS_INFO("---");
  ROS_INFO_STREAM("current_pose: " << turtle_.pose.value());
  ROS_INFO_STREAM("prev_turn_pose: " << prev_turn_pose_.value());
  ROS_INFO_STREAM(
      "distance from last turning position: " << calc_distance(turtle_.pose.value(), prev_turn_pose_.value()));
  ROS_INFO_STREAM(
      "diff from target direction: " << abs(
          calc_target_direction(turtle_.pose.value(), turn_count_) - turtle_.pose.value().theta));
}
