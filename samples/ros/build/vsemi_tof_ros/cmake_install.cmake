# Install script for directory: /home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/src/vsemi_tof_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/vsemi_tof_ros" TYPE FILE FILES "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/devel/include/vsemi_tof_ros/vsemi_tof_rosConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/vsemi_tof_ros" TYPE FILE FILES "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/devel/lib/python2.7/dist-packages/vsemi_tof_ros/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/devel/lib/python2.7/dist-packages/vsemi_tof_ros/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/vsemi_tof_ros" TYPE DIRECTORY FILES "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/devel/lib/python2.7/dist-packages/vsemi_tof_ros/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/build/vsemi_tof_ros/catkin_generated/installspace/vsemi_tof_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vsemi_tof_ros/cmake" TYPE FILE FILES
    "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/build/vsemi_tof_ros/catkin_generated/installspace/vsemi_tof_rosConfig.cmake"
    "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/build/vsemi_tof_ros/catkin_generated/installspace/vsemi_tof_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vsemi_tof_ros" TYPE FILE FILES "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/src/vsemi_tof_ros/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros/tof_cam_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros/tof_cam_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros/tof_cam_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros" TYPE EXECUTABLE FILES "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/devel/lib/vsemi_tof_ros/tof_cam_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros/tof_cam_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros/tof_cam_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros/tof_cam_node"
         OLD_RPATH "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/src/vsemi_tof_ros/../../../../driver/lib/x86_64:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vsemi_tof_ros/tof_cam_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vsemi_tof_ros" TYPE DIRECTORY FILES "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/src/vsemi_tof_ros/launch")
endif()

