cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_VDEBUG})
    add_executable(detector_vdebug vdebug.cc)
    target_link_libraries(detector_vdebug
      common
      communication
      detection
      sensors
      vehicles
      ${OPENGL_LIBRARIES}
      ${GLUT_LIBRARIES}
    )
endif()
