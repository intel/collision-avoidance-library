cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

configure_file(coav-control.sh.in coav-control.sh @ONLY)

if(DEFINED ENV{SDKTARGETSYSROOT})
    install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/coav-control.sh
        DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/init.d)
endif()

set(LIBRARIES coav)

if (${WITH_VDEBUG})
    set(LIBRARIES ${LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
endif()

add_executable(coav-control coav-control.cc parser.cc visual.cc visual_depth.cc)
target_link_libraries(coav-control ${LIBRARIES})
install(TARGETS coav-control
    DESTINATION ${CMAKE_INSTALL_BINDIR})
