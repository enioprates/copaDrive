# Install script for directory: /home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing/msg" TYPE FILE FILES
    "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/msg/coords.msg"
    "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/msg/drive_param.msg"
    "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/msg/AckermannDriveStamped.msg"
    "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/msg/error_control.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing/cmake" TYPE FILE FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/build/image_processing/catkin_generated/installspace/image_processing-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/devel/include/image_processing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/devel/share/roseus/ros/image_processing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/devel/share/common-lisp/ros/image_processing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/devel/share/gennodejs/ros/image_processing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/devel/lib/python2.7/dist-packages/image_processing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/devel/lib/python2.7/dist-packages/image_processing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/build/image_processing/catkin_generated/installspace/image_processing.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing/cmake" TYPE FILE FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/build/image_processing/catkin_generated/installspace/image_processing-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing/cmake" TYPE FILE FILES
    "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/build/image_processing/catkin_generated/installspace/image_processingConfig.cmake"
    "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/build/image_processing/catkin_generated/installspace/image_processingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing" TYPE FILE FILES "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/package.xml")
endif()

