#include "turtlesim_controller/turtlesim_controller.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "turtlesim_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TurtlesimController turtlesim_controller(nh, pnh);

  return 0;
}
