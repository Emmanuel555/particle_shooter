cmake_minimum_required(VERSION 2.8.3)
project(particle_shooter)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  std_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For particle_shooter_plugin plugin
add_library(particle_shooter_plugin src/particle_shooter_plugin.cpp)
target_link_libraries(particle_shooter_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    std_msgs 
)
