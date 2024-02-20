# Install script for directory: /home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs/msg" TYPE FILE FILES
    "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg"
    "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs/cmake" TYPE FILE FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/devel/include/mtt_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/devel/share/roseus/ros/mtt_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/devel/share/common-lisp/ros/mtt_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/devel/share/gennodejs/ros/mtt_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/devel/lib/python2.7/dist-packages/mtt_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/devel/lib/python2.7/dist-packages/mtt_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs/cmake" TYPE FILE FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs/cmake" TYPE FILE FILES
    "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgsConfig.cmake"
    "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs" TYPE FILE FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs/cmake" TYPE FILE FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs/cmake" TYPE FILE FILES
    "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgsConfig.cmake"
    "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/build/mtt_msgs/catkin_generated/installspace/mtt_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtt_msgs" TYPE FILE FILES "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/package.xml")
endif()

