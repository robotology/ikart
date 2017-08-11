#
# Copyright (C) 2012 RobotCub Consortium
# Author: Vadim Tikhanoff vadim.tikhanoff@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(appname iKartWirelessDisplay)

file(GLOB conf     ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB pictures ${CMAKE_CURRENT_SOURCE_DIR}/pictures/*.bmp)

yarp_install(FILES ${conf}     DESTINATION ${IKART_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${pictures} DESTINATION ${IKART_CONTEXTS_INSTALL_DIR}/${appname}/pictures)

