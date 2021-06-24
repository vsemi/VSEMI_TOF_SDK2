cmake_minimum_required(VERSION 3.5.0)

PROJECT(vsemi_tof_pcl)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

message( STATUS "Architecture: ${ARCHITECTURE}" )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)

message("OpenCV version: ${OpenCV_VERSION}")

find_package (Threads)

find_package(PCL REQUIRED COMPONENTS common io visualization)

file(GLOB app_INCLUDE_DIR
    "../../driver/include"
)

file(GLOB app_LIB_DIR
    "../../driver/lib/${ARCHITECTURE}"
)

file(GLOB app_SRC
    "src/*.cpp"
    "src/*.h"
)

include_directories( ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${app_INCLUDE_DIR})

link_directories( ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${app_LIB_DIR})

add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(camera ${app_SRC})

TARGET_LINK_LIBRARIES(camera ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} vsemi_tof_cam)

set_property(TARGET camera PROPERTY CXX_STANDARD 11)

