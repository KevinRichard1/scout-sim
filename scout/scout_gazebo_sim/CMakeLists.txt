cmake_minimum_required(VERSION 3.5)
project(scout_gazebo_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)

###########
## Build ##
###########

# Include directories for this project
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Create a static library
add_library(scout_gazebo STATIC src/scout_skid_steer.cpp)

ament_target_dependencies(scout_gazebo
  rclcpp
  geometry_msgs
  sensor_msgs
  gazebo_ros
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Create an executable for the skid steer controller
add_executable(scout_skid_steer_controller src/scout_skid_steer_controller.cpp)
target_link_libraries(scout_skid_steer_controller scout_gazebo)
ament_target_dependencies(scout_skid_steer_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  gazebo_ros
)

#############
## Install ##
#############

# Specify where to install files
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()