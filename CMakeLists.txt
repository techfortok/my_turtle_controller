cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_controller)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  turtlesim
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
