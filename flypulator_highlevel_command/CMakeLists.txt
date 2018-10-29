cmake_minimum_required(VERSION 2.8.3)
project(flypulator_highlevel_command)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  hl_command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

 add_executable(hl_command_server src/hl_command_server.cpp)
add_dependencies(hl_command_server flypulator_highlevel_command_gencpp)
target_link_libraries(hl_command_server ${catkin_LIBRARIES})