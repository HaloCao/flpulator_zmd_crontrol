cmake_minimum_required(VERSION 2.8.3)
project(flypulator_launch)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  flypulator_description 
  flypulator_common_msgs 
  flypulator_gazebo_plugin 
  flypulator_traj_generator 
  flypulator_teleoperation 
  flypulator_control
  hector_gazebo_plugins 
  flypulator_imp_control
)
