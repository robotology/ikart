#
# Copyright (C) 2017 IIT - iCub Facility
# Author: Marco Randazzo marco.randazzo@iit.it, Francesco Rea francesco.rea@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

file(GLOB maps    ${CMAKE_CURRENT_SOURCE_DIR}/*.rar)

yarp_install(FILES ${maps}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/iKartMaps)

add_custom_command(OUTPUT *.*
                   COMMAND ${CMAKE_COMMAND} -E unrar e *.rar
                   DEPENDS *.rar
                   WORKING_DIRECTORY ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/iKartMaps
                   COMMENT "Unpacking path/to/file.tar.gz"
                   VERBATIM)



