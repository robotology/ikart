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
