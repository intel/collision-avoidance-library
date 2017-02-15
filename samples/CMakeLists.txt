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

if (${WITH_REALSENSE})
    add_executable(polarhist_sample polarhist_sample.cc)
    target_link_libraries(polarhist_sample
      common
      sensors
      detection
    )
endif()
