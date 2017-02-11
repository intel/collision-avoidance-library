cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_TOOLS})
    configure_file(coav-control.sh.in coav-control.sh @ONLY)

    if(DEFINED ENV{SDKTARGETSYSROOT})
        install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/coav-control.sh
                DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/init.d)
    endif()

    add_executable(coav-control coav-control.cc parser.cc)
    target_link_libraries(coav-control
      common
      sensors
      detection
      avoidance
      vehicles
    )
    install(TARGETS coav-control
            DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
endif()
