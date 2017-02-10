cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_GAZEBO})
    add_executable(coav_sample_app coav_sample_app.cc)
    target_link_libraries(coav_sample_app
      common
      sensors
      detection
      avoidance
      vehicles
    )
endif()

if (${WITH_VDEBUG})
    add_executable(test_detector test.cc)
    target_link_libraries(test_detector
      common
      communication
      detection
      sensors
      vehicles
      ${OPENGL_LIBRARIES}
      ${GLUT_LIBRARIES}
    )
endif()

if (${WITH_REALSENSE})
    add_executable(polarhist_test polarhist_test.cc)
    target_link_libraries(polarhist_test
      common
      sensors
      detection
    )
endif()
