cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

add_executable(coav_gcs coav_gcs.cc)
add_executable(test_detector test.cc)
add_executable(simple_gcs simple_gcs.cc)

target_link_libraries(simple_gcs
  common
  sensors
  detection
  avoidance
  vehicles
)

target_link_libraries(coav_gcs
  common
  sensors
  detection
  avoidance
  vehicles
)

target_link_libraries(test_detector
  common
  communication
  detection
  sensors
  vehicles
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

if(REALSENSE_FOUND)
    add_definitions(-DUSING_REALSENSE)
endif(REALSENSE_FOUND)

