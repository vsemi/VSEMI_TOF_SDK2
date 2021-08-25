cmake_minimum_required(VERSION 3.5.0)

PROJECT(camera_info_reader)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

message( STATUS "Architecture: ${ARCHITECTURE}" )

file(GLOB app_INCLUDE_DIR
    "../../driver/include"
)

file(GLOB app_LIB_DIR
    "../../driver/lib/${ARCHITECTURE}"
)

file(GLOB app_SRC
    "src/*.cpp"
)

include_directories( ${app_INCLUDE_DIR} )

link_directories( ${app_LIB_DIR} )

ADD_EXECUTABLE( camera_info_reader ${app_SRC})

TARGET_LINK_LIBRARIES( camera_info_reader vsemi_tof_cam )

#set_property(TARGET id_read PROPERTY CXX_STANDARD 98)

