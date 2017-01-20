cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_TOOLS})
    add_executable(coav-control coav-control.cc parser.cc)
    target_link_libraries(coav-control
      common
      sensors
      detection
      avoidance
      vehicles
    )
endif()
