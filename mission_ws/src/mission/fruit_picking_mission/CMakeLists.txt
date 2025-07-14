cmake_minimum_required(VERSION 3.0.2)
project(fruit_picking_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  user_msgs
  actionlib
  actionlib_msgs
)

add_action_files(
  FILES
  PickFruit.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fruit_picking_mission
  CATKIN_DEPENDS rospy std_msgs user_msgs
#  DEPENDS system_lib
)

# Python scripts
catkin_install_python(PROGRAMS
  scripts/mission_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
