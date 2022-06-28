# Install script for directory: /home/paquitop/dot-paquitop/HighLevelSW/src/kinova/camera_robot_interaction

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/paquitop/dot-paquitop/HighLevelSW/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/camera_robot_interaction/srv" TYPE FILE FILES "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/camera_robot_interaction/srv/Movement.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/camera_robot_interaction/cmake" TYPE FILE FILES "/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/installspace/camera_robot_interaction-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/paquitop/dot-paquitop/HighLevelSW/devel/include/camera_robot_interaction")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/paquitop/dot-paquitop/HighLevelSW/devel/share/roseus/ros/camera_robot_interaction")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/paquitop/dot-paquitop/HighLevelSW/devel/share/common-lisp/ros/camera_robot_interaction")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/paquitop/dot-paquitop/HighLevelSW/devel/share/gennodejs/ros/camera_robot_interaction")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/paquitop/dot-paquitop/HighLevelSW/devel/lib/python2.7/dist-packages/camera_robot_interaction")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/paquitop/dot-paquitop/HighLevelSW/devel/lib/python2.7/dist-packages/camera_robot_interaction" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/paquitop/dot-paquitop/HighLevelSW/devel/lib/python2.7/dist-packages/camera_robot_interaction" FILES_MATCHING REGEX "/home/paquitop/dot-paquitop/HighLevelSW/devel/lib/python2.7/dist-packages/camera_robot_interaction/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/installspace/camera_robot_interaction.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/camera_robot_interaction/cmake" TYPE FILE FILES "/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/installspace/camera_robot_interaction-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/camera_robot_interaction/cmake" TYPE FILE FILES
    "/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/installspace/camera_robot_interactionConfig.cmake"
    "/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/installspace/camera_robot_interactionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/camera_robot_interaction" TYPE FILE FILES "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/camera_robot_interaction/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/camera_robot_interaction" TYPE PROGRAM FILES
    "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/camera_robot_interaction/scripts/robot_mover.py"
    "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/camera_robot_interaction/scripts/camera_client.py"
    "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/camera_robot_interaction/scripts/camera_client_obj_rec.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/camera_robot_interaction" TYPE DIRECTORY FILES "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/camera_robot_interaction/launch")
endif()

