# Install script for directory: /home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_ros_msgs/msg" TYPE FILE FILES
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/BagfileProgress.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/HistogramBucket.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/LandmarkEntry.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/LandmarkList.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/MetricFamily.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/MetricLabel.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/Metric.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/StatusCode.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/StatusResponse.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/SubmapEntry.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/SubmapList.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/SubmapTexture.msg"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/msg/TrajectoryStates.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_ros_msgs/srv" TYPE FILE FILES
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/srv/FinishTrajectory.srv"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/srv/GetTrajectoryStates.srv"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/srv/ReadMetrics.srv"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/srv/StartTrajectory.srv"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/srv/SubmapQuery.srv"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/srv/TrajectoryQuery.srv"
    "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/srv/WriteState.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_ros_msgs/cmake" TYPE FILE FILES "/home/orangepi/catkin_ws/build/algorithm/slam/Cartographer/cartographer_ros_msgs/catkin_generated/installspace/cartographer_ros_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/include/cartographer_ros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/share/roseus/ros/cartographer_ros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/share/common-lisp/ros/cartographer_ros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/share/gennodejs/ros/cartographer_ros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/orangepi/catkin_ws/devel/lib/python3/dist-packages/cartographer_ros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/orangepi/catkin_ws/devel/lib/python3/dist-packages/cartographer_ros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/orangepi/catkin_ws/build/algorithm/slam/Cartographer/cartographer_ros_msgs/catkin_generated/installspace/cartographer_ros_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_ros_msgs/cmake" TYPE FILE FILES "/home/orangepi/catkin_ws/build/algorithm/slam/Cartographer/cartographer_ros_msgs/catkin_generated/installspace/cartographer_ros_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_ros_msgs/cmake" TYPE FILE FILES
    "/home/orangepi/catkin_ws/build/algorithm/slam/Cartographer/cartographer_ros_msgs/catkin_generated/installspace/cartographer_ros_msgsConfig.cmake"
    "/home/orangepi/catkin_ws/build/algorithm/slam/Cartographer/cartographer_ros_msgs/catkin_generated/installspace/cartographer_ros_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_ros_msgs" TYPE FILE FILES "/home/orangepi/catkin_ws/src/algorithm/slam/Cartographer/cartographer_ros_msgs/package.xml")
endif()

