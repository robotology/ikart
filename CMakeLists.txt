#
# Copyright (C) 2017 IIT - iCub Facility
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

cmake_minimum_required(VERSION 2.6)

project(ikart)
if(WIN32)
    #disable annoying warnings about use of deprecated c functions
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    add_definitions(/wd4250)
endif()

find_package(YARP)
find_package(ICUB)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(ikart)
include(AddInstallRPATHSupport)

find_package(OpenCV)
find_package(GSL)
find_package(SDL)

add_subdirectory(src)
add_subdirectory(app)

