cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_GAZEBO} OR ${WITH_REALSENSE})
    add_executable(simple_gcs simple_gcs.cc)
    target_link_libraries(simple_gcs
      common
      sensors
      detection
      avoidance
      vehicles
    )

    add_executable(coav_gcs coav_gcs.cc)
    target_link_libraries(coav_gcs
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
