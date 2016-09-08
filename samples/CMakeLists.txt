cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(coav_gcs coav_gcs.cc)
add_executable(test_detector test.cc)

target_link_libraries(coav_gcs
  common
  sensors
  detection
  avoidance
  utils
  vehicles
  ${GAZEBO_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${LDFLAGS}
)

target_link_libraries(test_detector
  common
  communication
  detection
  utils
  sensors
  vehicles
  ${GAZEBO_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

