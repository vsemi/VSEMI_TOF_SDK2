# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;dynamic_reconfigure;message_runtime;sensor_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lvsemi_tof_cam".split(';') if "-lvsemi_tof_cam" != "" else []
PROJECT_NAME = "vsemi_tof_ros"
PROJECT_SPACE_DIR = "/home/vsemi/sdk/VSEMI_TOF_SDK2/samples/ros/install"
PROJECT_VERSION = "1.3.0"
