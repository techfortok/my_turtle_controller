cmake_minimum_required(VERSION 3.0.2)
project(turtle_controller_ros1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  turtlesim
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(turtlesim_controller src/turtlesim_controller.cpp)
add_dependencies(turtlesim_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlesim_controller ${catkin_LIBRARIES})
