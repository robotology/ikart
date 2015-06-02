#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(MODULENAME laserScannerGui)
project(${MODULENAME})

file(GLOB folder_source *.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_executable(${MODULENAME} ${folder_source})
target_link_libraries(${MODULENAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${MODULENAME} DESTINATION bin)
