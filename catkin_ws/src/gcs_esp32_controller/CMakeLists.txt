cmake_minimum_required(VERSION 3.0.2)
project(gcs_esp32_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

## Install Python executable scripts
# This command installs your Python script to a location where rosrun can find it.
# For scripts, a common destination is CATKIN_PACKAGE_SHARE_DESTINATION/scripts
# or CATKIN_PACKAGE_BIN_DESTINATION.
# Let's use CATKIN_PACKAGE_SHARE_DESTINATION/scripts for clarity with roslaunch.

# Find all python scripts in the scripts directory
file(GLOB PYTHON_SCRIPTS "scripts/*.py")

install(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  # ALTERNATIVE: DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Catkin package declaration
catkin_package(
  # NO_INCLUDE_DIRS # No C++ headers
  # NO_LIBRARIES    # No C++ libraries
  CATKIN_DEPENDS rospy geometry_msgs std_msgs # Runtime dependencies
  # DEPENDS system_lib
)
