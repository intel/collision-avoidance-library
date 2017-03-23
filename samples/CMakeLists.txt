cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_GAZEBO})
    add_executable(coav_sample_app coav_sample_app.cc)
    target_link_libraries(coav_sample_app coav)
endif()

if (${WITH_REALSENSE})
    add_executable(detector_sample detector_sample.cc)
    target_link_libraries(detector_sample coav)

    add_executable(polarhist_sample polarhist_sample.cc)
    target_link_libraries(polarhist_sample coav)
endif()
