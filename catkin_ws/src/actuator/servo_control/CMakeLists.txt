cmake_minimum_required(VERSION 3.0.2)
project(servo_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SetAngle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES servo_control
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 可执行脚本
catkin_install_python(PROGRAMS
  scripts/servo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
