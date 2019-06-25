cmake_minimum_required(VERSION 2.8)
project(flypulator_vive_pose)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    flypulator_common_msgs
)



include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS flypulator_common_msgs)

add_executable(vive_pose src/vive_pose.cpp)
target_link_libraries(vive_pose ${catkin_LIBRARIES})
add_dependencies(vive_pose ${catkin_EXPORTED_TARGETS})  
