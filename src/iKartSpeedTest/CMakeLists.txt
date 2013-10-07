#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME iKartSpeedTest)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)

include_directories(${CMAKE_SOURCE_DIR}
                    ${SDL_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${iCubDev_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ${SDL_LIBRARY} icubmod ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
