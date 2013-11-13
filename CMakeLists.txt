#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

cmake_minimum_required(VERSION 2.6)

project(iKart)
if(WIN32)
    #disable annoying warnings about use of deprecated c functions
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    add_definitions(/wd4250)
endif()

find_package(YARP)
find_package(ICUB)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)
include(YarpInstallationHelpers)

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

find_package(GtkMM)
find_package(OpenCV)
find_package(GSL)
find_package(SDL)

add_subdirectory(src)
add_subdirectory(app)

