#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME iKartWirelessDisplay)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

link_libraries(${GtkMM_LIBRARIES})
link_directories(${GtkMM_LIBRARY_DIR})
include_directories(${GtkMM_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} 
                      ${GtkMM_LINK_FLAGS}
                      ${GtkMM_LIBRARIES}
                      ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

