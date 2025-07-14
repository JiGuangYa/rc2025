# Install script for directory: /home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/orangepi/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/srv" TYPE FILE FILES
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/srv/CheckPoint.srv"
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/srv/CheckPose.srv"
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/srv/CheckPath.srv"
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/srv/FindValidPose.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/action" TYPE FILE FILES
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/action/GetPath.action"
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/action/ExePath.action"
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/action/Recovery.action"
    "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/action/MoveBase.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/GetPathAction.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/GetPathActionGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/GetPathActionResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/GetPathActionFeedback.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/GetPathGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/GetPathResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/GetPathFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/ExePathAction.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/ExePathActionGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/ExePathActionResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/ExePathActionFeedback.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/ExePathGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/ExePathResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/ExePathFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/RecoveryAction.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/RecoveryActionGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/RecoveryActionResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/RecoveryActionFeedback.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/RecoveryGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/RecoveryResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/RecoveryFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/MoveBaseAction.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/MoveBaseActionGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/MoveBaseActionResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/MoveBaseActionFeedback.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/MoveBaseGoal.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/MoveBaseResult.msg"
    "/home/orangepi/catkin_ws/devel/share/mbf_msgs/msg/MoveBaseFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/cmake" TYPE FILE FILES "/home/orangepi/catkin_ws/build/algorithm/navigation/navigation/move_base_flex/mbf_msgs/catkin_generated/installspace/mbf_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/include/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/share/roseus/ros/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/share/common-lisp/ros/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/share/gennodejs/ros/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/orangepi/catkin_ws/devel/lib/python3/dist-packages/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/lib/python3/dist-packages/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/orangepi/catkin_ws/build/algorithm/navigation/navigation/move_base_flex/mbf_msgs/catkin_generated/installspace/mbf_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/cmake" TYPE FILE FILES "/home/orangepi/catkin_ws/build/algorithm/navigation/navigation/move_base_flex/mbf_msgs/catkin_generated/installspace/mbf_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/cmake" TYPE FILE FILES
    "/home/orangepi/catkin_ws/build/algorithm/navigation/navigation/move_base_flex/mbf_msgs/catkin_generated/installspace/mbf_msgsConfig.cmake"
    "/home/orangepi/catkin_ws/build/algorithm/navigation/navigation/move_base_flex/mbf_msgs/catkin_generated/installspace/mbf_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs" TYPE FILE FILES "/home/orangepi/catkin_ws/src/algorithm/navigation/navigation/move_base_flex/mbf_msgs/package.xml")
endif()

