#include <rclcpp/rclcpp.hpp>

#include "turtlesim_controller/turtlesim_controller.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimController>());
  rclcpp::shutdown();

  return 0;
}