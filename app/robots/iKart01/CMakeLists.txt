#
# Copyright (C) 2017 IIT - iCub Facility
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(appname iKart01)

set(scripts general.xml ikart_laserOnly.xml ikart_main.xml yarprobotinterface.ini yarpmotorgui.ini)

yarp_install(FILES ${scripts} DESTINATION ${IKART_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY calibrators DESTINATION ${IKART_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY wrappers DESTINATION ${IKART_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY hardware DESTINATION ${IKART_ROBOTS_INSTALL_DIR}/${appname})


