set(appname iKartNavigation)

file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB scripts   ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${IKART_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${templates} DESTINATION ${IKART_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${scripts}   DESTINATION ${IKART_APPLICATIONS_INSTALL_DIR})