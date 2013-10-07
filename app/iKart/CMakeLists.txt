#
# Copyright (C) 2010 RobotCub Consortium
# Author: Francesco Rea francesco.rea@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB maps    ${CMAKE_CURRENT_SOURCE_DIR}/maps/*.rar)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/iKart)
yarp_install(FILES ${maps}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/iKart/maps)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

add_custom_command(OUTPUT *.*
                   COMMAND ${CMAKE_COMMAND} -E unrar e *.rar
                   DEPENDS *.rar
                   WORKING_DIRECTORY ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/iKart/maps
                   COMMENT "Unpacking path/to/file.tar.gz"
                   VERBATIM)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ikart_boot/ikart_start.sh
              ${CMAKE_CURRENT_SOURCE_DIR}/ikart_boot/ikart_stop.sh
              ${CMAKE_CURRENT_SOURCE_DIR}/ikart_boot/ikart_motors_run.sh 
              ${CMAKE_CURRENT_SOURCE_DIR}/ikart_boot/ikart_motors_idle.sh
              DESTINATION bin 
              COMPONENT Core
              PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

