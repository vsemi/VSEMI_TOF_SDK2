cmake_minimum_required(VERSION 3.5.0)

PROJECT(app)

SET(TARGET_PLATFORM "x64_ubuntu" CACHE STRING "Target platform for Vsemi ToF Driver")

message("TARGET_PLATFORM set to ${TARGET_PLATFORM}")

file(GLOB app_INCLUDE_DIR
    "../driver/include"
)

file(GLOB app_LIB_DIR
    "../driver/lib/${TARGET_PLATFORM}"
)

file(GLOB app_SRC
    "src/*.cpp"
)

include_directories( ${app_INCLUDE_DIR} )

link_directories( ${app_LIB_DIR} )

ADD_EXECUTABLE( vcam ${app_SRC})

TARGET_LINK_LIBRARIES( vcam vsemi_tof_cam )

set_property(TARGET vcam PROPERTY CXX_STANDARD 98)

