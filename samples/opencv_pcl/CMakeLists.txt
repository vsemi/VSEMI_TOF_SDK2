cmake_minimum_required(VERSION 3.5.0)

PROJECT(vsemi_tof_pcl)

SET(TARGET_PLATFORM "x64_ubuntu" CACHE STRING "Target platform for Vsemi ToF Driver")

message("TARGET_PLATFORM set to ${TARGET_PLATFORM}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)

message("OpenCV version: ${OpenCV_VERSION}")

find_package (Threads)

find_package(PCL REQUIRED COMPONENTS common io visualization)

file(GLOB app_INCLUDE_DIR
    "../../driver/include"
)

file(GLOB app_LIB_DIR
    "../../driver/lib/${TARGET_PLATFORM}"
)

file(GLOB app_SRC
    "src/*.cpp"
    "src/*.h"
)

include_directories( ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${app_INCLUDE_DIR})

link_directories( ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${app_LIB_DIR})

add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(vcam ${app_SRC})

TARGET_LINK_LIBRARIES(vcam ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} vsemi_tof_cam)

set_property(TARGET vcam PROPERTY CXX_STANDARD 11)

