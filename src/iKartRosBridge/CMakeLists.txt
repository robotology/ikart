#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

cmake_minimum_required (VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(YARP)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
set_contrib_default_prefix()

include_directories(${YARP_INCLUDE_DIRS})

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rospack(ikart)
#genmsg()

rosbuild_init()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_executable(iKartRosBridge main.cpp bridgeThread.cpp)

target_link_libraries(iKartRosBridge ${YARP_LIBRARIES})



