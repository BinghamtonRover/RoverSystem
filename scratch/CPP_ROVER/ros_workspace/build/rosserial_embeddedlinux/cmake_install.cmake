# Install script for directory: /home/IEEE_PI/ros_workspace/src/rosserial_embeddedlinux

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/IEEE_PI/ros_workspace/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/IEEE_PI/ros_workspace/build/rosserial_embeddedlinux/catkin_generated/installspace/rosserial_embeddedlinux.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_embeddedlinux/cmake" TYPE FILE FILES
    "/home/IEEE_PI/ros_workspace/build/rosserial_embeddedlinux/catkin_generated/installspace/rosserial_embeddedlinuxConfig.cmake"
    "/home/IEEE_PI/ros_workspace/build/rosserial_embeddedlinux/catkin_generated/installspace/rosserial_embeddedlinuxConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_embeddedlinux" TYPE FILE FILES "/home/IEEE_PI/ros_workspace/src/rosserial_embeddedlinux/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_embeddedlinux/src" TYPE DIRECTORY FILES
    "/home/IEEE_PI/ros_workspace/src/rosserial_embeddedlinux/src/ros_lib"
    "/home/IEEE_PI/ros_workspace/src/rosserial_embeddedlinux/src/examples"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rosserial_embeddedlinux" TYPE PROGRAM FILES "/home/IEEE_PI/ros_workspace/build/rosserial_embeddedlinux/catkin_generated/installspace/make_libraries.py")
endif()

