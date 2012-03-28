#
# Copyright (C) 2010 RobotCub Consortium
# Author: Francesco Rea francesco.rea@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(appname iKart)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

icub_app_install(${appname}
		FILES ${conf}
		DESTINATION conf)

icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ikart_boot/ikart_start.sh ${CMAKE_CURRENT_SOURCE_DIR}/ikart_boot/ikart_stop.sh 
        DESTINATION bin 
        COMPONENT Core
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
