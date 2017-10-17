#
# Copyright (C) 2017 IIT - iCub Facility 
# Author: Francesco Rea francesco.rea@iit.it, Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ikart_start.sh
              ${CMAKE_CURRENT_SOURCE_DIR}/ikart_stop.sh
              ${CMAKE_CURRENT_SOURCE_DIR}/ikart_motors_run.sh 
              ${CMAKE_CURRENT_SOURCE_DIR}/ikart_motors_idle.sh
              DESTINATION bin 
              COMPONENT Core
              PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

