cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_lissajous)

# This is the ROS build system configuration
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
)

# Declare a catkin package
catkin_package()

# Include directories (for headers or any shared resources)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python scripts don't require compilation, just installation.
catkin_install_python(PROGRAMS
  scripts/lissajous_drawer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install any necessary directories (for configuration or data files)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
